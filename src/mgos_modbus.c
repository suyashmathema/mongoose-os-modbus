/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mgos_modbus.h"

#include "crc16.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define lowByte(w) ((uint8_t)((w)&0xff))
#define highByte(w) ((uint8_t)((w) >> 8))

enum uart_read_states { DISABLED,
                        READ_START,
                        RESP_METADATA,
                        RESP_COMPLETE };

enum MB_VALUE_TYPE {
    TYPE_FLOAT_32,
    TYPE_LONG_INV_32
};

struct mgos_modbus {
    struct mbuf receive_buffer;
    struct mbuf transmit_buffer;
    mb_response_callback cb;
    void* cb_arg;
    int uart_no;
    uint8_t slave_id_u8;
    uint16_t read_address_u16;
    uint16_t read_qty_u16;
    uint16_t write_address_u16;
    uint16_t write_qty_u16;
    uint8_t* write_data;
    uint8_t write_data_len;
    uint8_t mask_and;
    uint8_t mask_or;
    uint8_t func_code_u8;
    uint8_t resp_status_u8;
    uint8_t resp_bytes_u8;
    enum uart_read_states read_state;
};

static struct mgos_modbus* s_modbus = NULL;
static mgos_timer_id req_timer;

static void print_buffer(struct mbuf buffer) {
    char str[1024];
    int length = 0;
    for (int i = 0; i < buffer.len && i < sizeof(str) / 3; i++) {
        length += sprintf(str + length, "%.2x ", buffer.buf[i]);
    }
    LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Buffer: %.*s", s_modbus->slave_id_u8, s_modbus->func_code_u8, length, str));
}

static size_t mbuf_append_16(struct mbuf* buffer, uint16_t value) {
    size_t size = 0;
    uint8_t lb = lowByte(value);
    uint8_t hb = highByte(value);
    size = mbuf_append(buffer, &hb, sizeof(uint8_t));
    size += mbuf_append(buffer, &lb, sizeof(uint8_t));
    return size;
}

static uint16_t calculate_crc16(struct mbuf value) {
    uint16_t u16CRC = 0xFFFF;
    for (int i = 0; i < value.len; i++) {
        u16CRC = crc16_update(u16CRC, (uint8_t)value.buf[i]);
    }

    return (lowByte(u16CRC) << 8) | (highByte(u16CRC) & 0xff);  //CRC Lower Byte first then Higher Byte
}

static uint8_t verify_crc16(struct mbuf value) {
    value.len -= 2;
    uint16_t u16CRC = calculate_crc16(value);
    if (highByte(u16CRC) != value.buf[value.len] || lowByte(u16CRC) != value.buf[value.len + 1]) {  //Since calculate returns lower byte first
        return RESP_INVALID_CRC;
    }
    return RESP_SUCCESS;
}

/*
Callback function that is called when a set timeout period expires before receiving a response.
*/
static void req_timeout_cb(void* arg) {
    LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Request timed out", s_modbus->slave_id_u8, s_modbus->func_code_u8));
    s_modbus->resp_status_u8 = RESP_TIMED_OUT;
    struct mb_request_info ri = {
        s_modbus->slave_id_u8,
        s_modbus->read_address_u16,
        s_modbus->read_qty_u16,
        s_modbus->write_address_u16,
        s_modbus->write_qty_u16,
        s_modbus->mask_and,
        s_modbus->mask_or,
        s_modbus->func_code_u8,
    };
    s_modbus->cb(RESP_TIMED_OUT, ri, s_modbus->receive_buffer, s_modbus->cb_arg);
    s_modbus->read_state = DISABLED;
    req_timer = 0;
    (void)arg;
}

/*
To validate slave id, function code and modbus exception in the response and
estimate size of the requested data from the response.
*/
static bool validate_mb_metadata(struct mbuf* buffer) {
    // verify response is for correct Modbus slave
    if ((uint8_t)buffer->buf[0] != s_modbus->slave_id_u8) {
        s_modbus->resp_status_u8 = RESP_INVALID_SLAVE_ID;
    }
    // verify response is for correct Modbus function code (mask exception bit 7)
    if (((uint8_t)buffer->buf[1] & 0x7F) != s_modbus->func_code_u8) {
        s_modbus->resp_status_u8 = RESP_INVALID_FUNCTION;
    }
    // check whether Modbus exception occurred; return Modbus Exception Code
    if (bitRead((uint8_t)buffer->buf[1], 7)) {
        s_modbus->resp_status_u8 = (uint8_t)buffer->buf[2];
    }

    if (s_modbus->resp_status_u8) {
        return false;
    }

    // evaluate returned Modbus function code to get modbus requested data size
    uint8_t resp_func = buffer->buf[1];
    if (resp_func == FUNC_READ_COILS ||
        resp_func == FUNC_READ_DISCRETE_INPUTS ||
        resp_func == FUNC_READ_INPUT_REGISTERS ||
        resp_func == FUNC_READ_HOLDING_REGISTERS ||
        resp_func == FUNC_READ_WRITE_MULTIPLE_REGISTERS) {
        s_modbus->resp_bytes_u8 = 5 + (uint8_t)buffer->buf[2];
    } else if (resp_func == FUNC_WRITE_SINGLE_COIL ||
               resp_func == FUNC_WRITE_MULTIPLE_COILS ||
               resp_func == FUNC_WRITE_SINGLE_REGISTER ||
               resp_func == FUNC_WRITE_MULTIPLE_REGISTERS) {
        s_modbus->resp_bytes_u8 = 8;
    } else if (resp_func == FUNC_MASK_WRITE_REGISTER) {
        s_modbus->resp_bytes_u8 = 10;
    }
    return true;
}

static void update_modbus_read_state(struct mbuf* buffer) {
    struct mb_request_info ri = {
        s_modbus->slave_id_u8,
        s_modbus->read_address_u16,
        s_modbus->read_qty_u16,
        s_modbus->write_address_u16,
        s_modbus->write_qty_u16,
        s_modbus->mask_and,
        s_modbus->mask_or,
        s_modbus->func_code_u8,
    };

    switch (s_modbus->read_state) {
        case DISABLED:
            /*
    Do not disable RX on default condition similar to RS485 control.
    Buffer piles up with grabage values once it is enabled. Just discard
    any values received while not expecting any response.
    */
            mbuf_clear(buffer);
            return;
        case READ_START:
            LOG(LL_VERBOSE_DEBUG, ("SlaveID: %.2x, Function: %.2x - Read modbus response start", s_modbus->slave_id_u8, s_modbus->func_code_u8));
            int count = 0;
            for (int i = 0; i < buffer->len; i++) {
                if (buffer->buf[i] != s_modbus->slave_id_u8) {
                    count++;
                } else {
                    mbuf_remove(buffer, count);
                    s_modbus->read_state = RESP_METADATA;
                    update_modbus_read_state(buffer);
                    return;
                }
            }
            mbuf_remove(buffer, count);
            return;
        case RESP_METADATA:
            if (buffer->len < s_modbus->resp_bytes_u8) {
                return;
            }
            if (!validate_mb_metadata(buffer)) {
                LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Invalid Response: %.2x, SlaveID: %.2x, Function: %.2x",
                               s_modbus->slave_id_u8, s_modbus->func_code_u8, s_modbus->resp_status_u8,
                               (uint8_t)buffer->buf[0], (uint8_t)buffer->buf[1]));
                break;
            }
            s_modbus->read_state = RESP_COMPLETE;
            update_modbus_read_state(buffer);
            return;
        case RESP_COMPLETE:
            if (buffer->len < s_modbus->resp_bytes_u8) {
                return;
            }
            buffer->len = s_modbus->resp_bytes_u8;
            if (verify_crc16(*buffer) == RESP_INVALID_CRC) {
                LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Invalid CRC", s_modbus->slave_id_u8, s_modbus->func_code_u8));
                s_modbus->resp_status_u8 = RESP_INVALID_CRC;
                break;
            }

            LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Modbus response received %d",
                           s_modbus->slave_id_u8, s_modbus->func_code_u8, s_modbus->receive_buffer.len));
            s_modbus->resp_status_u8 = RESP_SUCCESS;
            break;
        default:
            return;
    }
    print_buffer(s_modbus->receive_buffer);
    mgos_clear_timer(req_timer);
    s_modbus->cb(s_modbus->resp_status_u8, ri, s_modbus->receive_buffer, s_modbus->cb_arg);
    s_modbus->read_state = DISABLED;
}

static void uart_cb(int uart_no, void* param) {
    (void)param;
    assert(uart_no == s_modbus->uart_no);
    struct mbuf* buffer = &s_modbus->receive_buffer;

    size_t rx_av = mgos_uart_read_avail(uart_no);
    if (rx_av == 0) {
        return;
    }

    mgos_uart_read_mbuf(uart_no, buffer, rx_av);
    LOG(LL_VERBOSE_DEBUG, ("SlaveID: %.2x, Function: %.2x - uart_cb - Receive Buffer: %d, Read Available: %d",
                           s_modbus->slave_id_u8, s_modbus->func_code_u8, s_modbus->receive_buffer.len, rx_av));
    update_modbus_read_state(buffer);
}

static bool init_modbus(uint8_t slave_id, uint8_t func_code, uint8_t total_resp_bytes, mb_response_callback cb, void* cb_arg) {
    if (s_modbus->read_state != DISABLED)
        return false;
    s_modbus->cb = cb;
    s_modbus->cb_arg = cb_arg;
    s_modbus->resp_bytes_u8 = total_resp_bytes;
    s_modbus->slave_id_u8 = slave_id;
    s_modbus->func_code_u8 = func_code;
    LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Initialize Modbus", s_modbus->slave_id_u8, s_modbus->func_code_u8));
    return true;
}

static void set_read_address(uint16_t read_address, uint16_t read_qty) {
    s_modbus->read_address_u16 = read_address;
    s_modbus->read_qty_u16 = read_qty;
}

static void set_write_address(uint16_t write_address, uint16_t write_qty, uint8_t* data, uint8_t len) {
    s_modbus->write_address_u16 = write_address;
    s_modbus->write_qty_u16 = write_qty;
    if (data != NULL || len > 0) {
        s_modbus->write_data = data;
        s_modbus->write_data_len = len;
    }
}

static size_t set_transmit_buffer() {
    mbuf_clear(&s_modbus->transmit_buffer);

    size_t append = mbuf_append(&s_modbus->transmit_buffer, &s_modbus->slave_id_u8, sizeof(uint8_t));
    append += mbuf_append(&s_modbus->transmit_buffer, &s_modbus->func_code_u8, sizeof(uint8_t));

    uint8_t func_code = s_modbus->func_code_u8;
    if (func_code == FUNC_READ_COILS ||
        func_code == FUNC_READ_DISCRETE_INPUTS ||
        func_code == FUNC_READ_INPUT_REGISTERS ||
        func_code == FUNC_READ_HOLDING_REGISTERS ||
        func_code == FUNC_READ_WRITE_MULTIPLE_REGISTERS) {
        append += mbuf_append_16(&s_modbus->transmit_buffer, s_modbus->read_address_u16);
        append += mbuf_append_16(&s_modbus->transmit_buffer, s_modbus->read_qty_u16);
    }

    if (func_code == FUNC_WRITE_SINGLE_COIL ||
        func_code == FUNC_WRITE_MULTIPLE_COILS ||
        func_code == FUNC_WRITE_SINGLE_REGISTER ||
        func_code == FUNC_WRITE_MULTIPLE_REGISTERS ||
        func_code == FUNC_READ_WRITE_MULTIPLE_REGISTERS) {
        append += mbuf_append_16(&s_modbus->transmit_buffer, s_modbus->write_address_u16);
        append += mbuf_append_16(&s_modbus->transmit_buffer, s_modbus->write_qty_u16);
    }

    if (func_code == FUNC_WRITE_MULTIPLE_COILS ||
        func_code == FUNC_WRITE_MULTIPLE_REGISTERS ||
        func_code == FUNC_READ_WRITE_MULTIPLE_REGISTERS) {
        append += mbuf_append(&s_modbus->transmit_buffer, &s_modbus->write_data_len, sizeof(uint8_t));
        append += mbuf_append_and_free(&s_modbus->transmit_buffer, s_modbus->write_data, s_modbus->write_data_len);
        LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Set Transmit Buffer", s_modbus->slave_id_u8, s_modbus->func_code_u8));
    }

    if (func_code == FUNC_MASK_WRITE_REGISTER) {
        append += mbuf_append_16(&s_modbus->transmit_buffer, s_modbus->read_address_u16);
        append += mbuf_append_16(&s_modbus->transmit_buffer, s_modbus->mask_and);
        append += mbuf_append_16(&s_modbus->transmit_buffer, s_modbus->mask_or);
    }

    append += mbuf_append_16(&s_modbus->transmit_buffer, calculate_crc16(s_modbus->transmit_buffer));

    LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Transmit Buffer %d",
                   s_modbus->slave_id_u8, s_modbus->func_code_u8, s_modbus->transmit_buffer.len));
    print_buffer(s_modbus->transmit_buffer);

    return append;
}

static bool start_transaction() {
    mbuf_clear(&s_modbus->receive_buffer);
    s_modbus->resp_status_u8 = 0x00;

    if (s_modbus->read_state == DISABLED && s_modbus->transmit_buffer.len > 0) {
        LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Modbus Transaction Start", s_modbus->slave_id_u8, s_modbus->func_code_u8));
        s_modbus->read_state = READ_START;
        mgos_uart_flush(s_modbus->uart_no);
        mgos_msleep(30);  //TODO delay for 3.5 Characters length according to baud rate
        req_timer = mgos_set_timer(mgos_sys_config_get_modbus_timeout(), 0, req_timeout_cb, NULL);
        mgos_uart_write(s_modbus->uart_no, s_modbus->transmit_buffer.buf, s_modbus->transmit_buffer.len);
        mgos_uart_set_rx_enabled(s_modbus->uart_no, true);
        mgos_uart_set_dispatcher(s_modbus->uart_no, uart_cb, &req_timer);
        return true;
    }
    return false;
}

/*
Read coils from modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_read_coils(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Read Coils, Address: %.2x", read_address));
    if (!init_modbus(slave_id, FUNC_READ_COILS, 5, cb, cb_arg))
        return false;
    set_read_address(read_address, read_qty);

    if (!set_transmit_buffer())
        return false;
    return start_transaction();
}

/*
Read discrete inputs from modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_read_discrete_inputs(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Read Discrete Inputs, Address: %.2x", read_address));
    if (!init_modbus(slave_id, FUNC_READ_DISCRETE_INPUTS, 5, cb, cb_arg))
        return false;
    set_read_address(read_address, read_qty);

    if (!set_transmit_buffer())
        return false;
    return start_transaction();
}

/*
Read holding registers from modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_read_holding_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Read Holding Registers, Address: %.2x", read_address));
    if (!init_modbus(slave_id, FUNC_READ_HOLDING_REGISTERS, 5, cb, cb_arg))
        return false;
    set_read_address(read_address, read_qty);

    if (!set_transmit_buffer())
        return false;
    return start_transaction();
}

/*
Read input registers from modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_read_input_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Read Input Registers, Address: %.2x", read_address));
    if (!init_modbus(slave_id, FUNC_READ_INPUT_REGISTERS, 5, cb, cb_arg))
        return false;
    set_read_address(read_address, read_qty);

    if (!set_transmit_buffer())
        return false;
    return start_transaction();
}

/*
Write coil in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_write_single_coil(uint8_t slave_id, uint16_t write_address, uint16_t write_value, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Write Single Coil, Address: %.2x", write_address));
    if (!init_modbus(slave_id, FUNC_WRITE_SINGLE_COIL, 4, cb, cb_arg))
        return false;
    set_write_address(write_address, write_value, NULL, 0);

    if (!set_transmit_buffer())
        return false;
    return start_transaction();
}

/*
Write register in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_write_single_register(uint8_t slave_id, uint16_t write_address, uint16_t write_value, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Write Single Register, Address: %.2x", write_address));
    if (!init_modbus(slave_id, FUNC_WRITE_SINGLE_REGISTER, 4, cb, cb_arg))
        return false;
    set_write_address(write_address, write_value, NULL, 0);

    if (!set_transmit_buffer())
        return false;
    return start_transaction();
}

/*
Write coils in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_write_multiple_coils(uint8_t slave_id, uint16_t write_address, uint16_t write_qty,
                             uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Write Multiple Coils, Address: %.2x", write_address));
    if (!init_modbus(slave_id, FUNC_WRITE_MULTIPLE_COILS, 4, cb, cb_arg))
        return false;
    set_write_address(write_address, write_qty, data, len);

    if (!set_transmit_buffer())
        return false;
    return start_transaction();
}

/*
Write registers in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_write_multiple_registers(uint8_t slave_id, uint16_t write_address, uint16_t write_qty,
                                 uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Write Multiple Registers, Address: %.2x", write_address));
    if (!init_modbus(slave_id, FUNC_WRITE_MULTIPLE_REGISTERS, 4, cb, cb_arg))
        return false;
    set_write_address(write_address, write_qty, data, len);

    if (!set_transmit_buffer())
        return false;
    return start_transaction();
}

/*
Read and write mulitple registers in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_read_write_multiple_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                                      uint16_t write_address, uint16_t write_qty,
                                      uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Read Write Multiple Registers, Address: %.2x", write_address));
    if (!init_modbus(slave_id, FUNC_READ_WRITE_MULTIPLE_REGISTERS, 4, cb, cb_arg))
        return false;
    set_read_address(read_address, read_qty);
    set_write_address(write_address, write_qty, data, len);
    if (!set_transmit_buffer())
        return false;
    return start_transaction();
}

/*
Mask register in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_mask_write_register(uint8_t slave_id, uint16_t address, uint16_t and_mask, uint16_t or_mask, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Mask Write Register, Address: %.2x", address));
    if (!init_modbus(slave_id, FUNC_MASK_WRITE_REGISTER, 4, cb, cb_arg))
        return false;
    s_modbus->read_address_u16 = address;
    s_modbus->mask_and = and_mask;
    s_modbus->mask_or = or_mask;

    if (!set_transmit_buffer())
        return false;
    return start_transaction();
}

bool mgos_modbus_create(const struct mgos_config_modbus* cfg) {
    struct mgos_uart_config ucfg;
    mgos_uart_config_set_defaults(cfg->uart_no, &ucfg);
    ucfg.baud_rate = mgos_sys_config_get_modbus_baudrate();
    if (mgos_sys_config_get_modbus_uart_rx_pin() >= 0) {
        ucfg.dev.rx_gpio = mgos_sys_config_get_modbus_uart_rx_pin();
    }
    if (mgos_sys_config_get_modbus_uart_tx_pin() >= 0) {
        ucfg.dev.tx_gpio = mgos_sys_config_get_modbus_uart_tx_pin();
    }
    if (mgos_sys_config_get_modbus_tx_en_gpio() >= 0) {
        ucfg.dev.tx_en_gpio = mgos_sys_config_get_modbus_tx_en_gpio();
    }
    if (mgos_sys_config_get_modbus_parity() >= 0 && mgos_sys_config_get_modbus_parity() < 3) {
        ucfg.parity = mgos_sys_config_get_modbus_parity();
    }
    if (mgos_sys_config_get_modbus_stop_bits() > 0 && mgos_sys_config_get_modbus_stop_bits() < 4) {
        ucfg.stop_bits = mgos_sys_config_get_modbus_stop_bits();
    }
    ucfg.dev.hd = mgos_sys_config_get_modbus_tx_en_enable();
    ucfg.dev.tx_en_gpio_val = mgos_sys_config_get_modbus_tx_en_gpio_val();

    char b1[8], b2[8], b3[8];
    LOG(LL_DEBUG, ("MODBUS UART%d (RX:%s TX:%s TX_EN:%s), Baudrate %d, Parity %d, Stop bits %d, Half Duplex %d, tx_en Value %d",
                   cfg->uart_no, mgos_gpio_str(ucfg.dev.rx_gpio, b1),
                   mgos_gpio_str(ucfg.dev.tx_gpio, b2),
                   mgos_gpio_str(ucfg.dev.tx_en_gpio, b3), ucfg.baud_rate,
                   ucfg.parity, ucfg.stop_bits, ucfg.dev.hd, ucfg.dev.tx_en_gpio_val));

    if (!mgos_uart_configure(cfg->uart_no, &ucfg)) {
        LOG(LL_ERROR, ("Failed to configure UART%d", cfg->uart_no));
        return false;
    }

    s_modbus = (struct mgos_modbus*)calloc(1, sizeof(*s_modbus));
    if (s_modbus == NULL)
        return false;
    mbuf_init(&s_modbus->transmit_buffer, 300);
    mbuf_init(&s_modbus->receive_buffer, 300);
    s_modbus->read_state = DISABLED;
    s_modbus->uart_no = cfg->uart_no;

    return true;
}

/*
  Assuming values from modbus are in sequence BADC
  example - Reading 2 holding registers starting at 156 for unit id 5
  request - 05 03 00 9c 00 02 05 a1
  response - 05 03 04 37(B) a8(A) 42(D) 48(C) 00 f1
*/

long parse_value_long_inverse_32(uint8_t* strt_ptr) {
    union {
        uint8_t c[4];
        long l;
    } u;
    //Setting modbus BADC to C memory BADC for long conversion
    u.c[3] = strt_ptr[0];  //B
    u.c[2] = strt_ptr[1];  //A
    u.c[1] = strt_ptr[2];  //D
    u.c[0] = strt_ptr[3];  //C
    return u.l;
}

float parse_value_float32(uint8_t* strt_ptr) {
    union {
        uint8_t c[4];
        float f;
    } u;
    //Setting modbus BADC to C memory DCBA for float conversion
    u.c[3] = strt_ptr[2];  //D
    u.c[2] = strt_ptr[3];  //C
    u.c[1] = strt_ptr[0];  //B
    u.c[0] = strt_ptr[1];  //A

    return u.f;
}

enum MB_VALUE_TYPE parse_address_info(struct json_token address_info, int* address) {
    *address = -1;
    enum MB_VALUE_TYPE type = TYPE_FLOAT_32;

    json_scanf(address_info.ptr, address_info.len, "%d", address);
    if (*address < 0) {
        char* type_temp = NULL;
        json_scanf(address_info.ptr, address_info.len, "{add: %d, type: %Q}", address, &type_temp);
        if (type_temp != NULL) {
            if (strcmp(type_temp, "float") == 0) {
                type = TYPE_FLOAT_32;
            } else if (strcmp(type_temp, "long_inv") == 0) {
                type = TYPE_LONG_INV_32;
            }
            free(type_temp);
        }
    }
    LOG(LL_DEBUG, ("Address Info, Value: %d, Type: %d", *address, type));
    return type;
}

int get_buffer_offset(uint16_t read_start_address, uint8_t byte_count, uint16_t required_address) {
    int read_qty = byte_count / 2;
    int max_read_address = read_start_address + read_qty - 1;
    if (required_address < read_start_address || required_address >= max_read_address) {
        LOG(LL_DEBUG, ("Invalid address, address out of range"));
        return -1;
    }
    int diff = required_address - read_start_address;
    if (diff % 2 != 0) {
        LOG(LL_DEBUG, ("Invalid address, address value not consistent"));
        return -1;
    }
    return diff * 2 + 3;
}

//Caller needs to free the returned attribute value string
char* get_attribute_value(struct mbuf* mb_reponse, uint16_t read_start_address, struct json_token attr_info) {
    int required_address = -1;
    enum MB_VALUE_TYPE type = parse_address_info(attr_info, &required_address);
    if (required_address < 0) {
        LOG(LL_DEBUG, ("Cannot find address in modbus response"));
        return NULL;
    }

    int offset = get_buffer_offset(read_start_address, (uint8_t)mb_reponse->buf[2], required_address);
    LOG(LL_DEBUG, ("Calculated offset for attribute: %d, address: %d, value type: %d",
                   offset, required_address, type));
    if (offset < 0) {
        LOG(LL_DEBUG, ("Invalid address"));
        return NULL;
    }
    uint8_t* start_position = (uint8_t*)mb_reponse->buf + offset;
    char* res = NULL;
    switch (type) {
        case TYPE_FLOAT_32:
            mg_asprintf(&res, 0, "%.2f", parse_value_float32(start_position));
            break;
        case TYPE_LONG_INV_32:
            mg_asprintf(&res, 0, "%ld", parse_value_long_inverse_32(start_position));
            break;
        default:
            break;
    }
    return res;
}

char* set_resp_json(struct mbuf* json_buf, char* offset, const char* key,
                    int key_len, const char* value, int value_len) {
    if (json_buf == NULL || key == NULL || value == NULL) {
        return offset;
    }
    int p = 0;
    if (json_buf->len == 0) {
        if ((p = mbuf_insert(json_buf, 0, "{}", 2)) <= 0) {
            return NULL;
        }
        offset = json_buf->buf + 1;
    } else {
        if ((p = mbuf_insert(json_buf, (int)(offset - json_buf->buf), ",", 1)) <= 0) {
            return NULL;
        }
        offset += p;
    }
    char* kv = NULL;
    mg_asprintf(&kv, 0, "\"%.*s\":%.*s", key_len, key, value_len, value);
    if (kv != NULL) {
        if ((p = mbuf_insert(json_buf, (int)(offset - json_buf->buf), kv, strlen(kv))) <= 0) {
            return NULL;
        }
        offset += p;
        free(kv);
    }
    return offset;
}

char* mb_map_response(const char* json_map, struct mbuf* mb_resp, struct mb_request_info* info) {
    LOG(LL_ERROR, ("Map modbus response to json"));
    void* h = NULL;
    char* offset = NULL;
    struct json_token attr_name, attr_info;
    struct mbuf resp_buf;
    mbuf_init(&resp_buf, strlen(json_map) * 2);
    while ((h = json_next_key(json_map, strlen(json_map), h, ".", &attr_name, &attr_info)) != NULL) {
        char* attr_value = NULL;
        if ((attr_value = get_attribute_value(mb_resp, info->read_address, attr_info)) != NULL) {
            if ((offset = set_resp_json(&resp_buf, offset, attr_name.ptr, attr_name.len,
                                        attr_value, strlen(attr_value))) == NULL) {
                LOG(LL_ERROR, ("Unable to create modbus mapped response json"));
                mbuf_free(&resp_buf);
                return NULL;
            }
            free(attr_value);
        }
    }
    char* resp = strndup(resp_buf.buf, resp_buf.len);
    mbuf_free(&resp_buf);
    return resp;
}

char* mb_map_responsef(const char* json_file, struct mbuf* mb_resp, struct mb_request_info* info) {
    char* map_str = json_fread(json_file);
    if (map_str == NULL) {
        LOG(LL_ERROR, ("Error reading modbus json map file"));
        return NULL;
    }
    char* resp = mb_map_response(map_str, mb_resp, info);
    free(map_str);
    return resp;
}

bool mgos_modbus_connect() {
    struct mgos_config_modbus* cfg = &mgos_sys_config.modbus;
    struct mgos_uart_config ucfg;
    char b1[8], b2[8], b3[8];
    mgos_uart_config_get(mgos_sys_config.modbus.uart_no, &ucfg);
    if (!mgos_uart_configure(cfg->uart_no, &ucfg)) {
        LOG(LL_ERROR, ("Failed to configure UART%d", cfg->uart_no));
        return false;
    }
    LOG(LL_DEBUG, ("MODBUS UART%d (RX:%s TX:%s TX_EN:%s), Baudrate %d, Parity %d, Stop bits %d, Half Duplex %d, tx_en Value %d",
                   cfg->uart_no, mgos_gpio_str(ucfg.dev.rx_gpio, b1),
                   mgos_gpio_str(ucfg.dev.tx_gpio, b2),
                   mgos_gpio_str(ucfg.dev.tx_en_gpio, b3), ucfg.baud_rate,
                   ucfg.parity, ucfg.stop_bits, ucfg.dev.hd, ucfg.dev.tx_en_gpio_val));
    return true;
}

bool mgos_modbus_init(void) {
    LOG(LL_DEBUG, ("Initializing modbus"));
    if (!mgos_sys_config_get_modbus_enable())
        return true;
    if (!mgos_modbus_create(&mgos_sys_config.modbus)) {
        return false;
    }
    return true;
}
