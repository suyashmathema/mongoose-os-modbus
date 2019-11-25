#include <stdint.h>
#include <stdlib.h>

#include "crc16.h"
#include "mgos.h"
#include "mgos_timers.h"
#include "mgos_uart.h"
#include "mgos_modbus.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define lowByte(w) ((uint8_t)((w)&0xff))
#define highByte(w) ((uint8_t)((w) >> 8))

struct mgos_modbus {
  struct mbuf receive_buffer;
  struct mbuf transmit_buffer;
  mb_response_callback cb;
  void* cb_arg;
  int uart_no;
  uint32_t start_time_u32;
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
  char state; //0: Waiting for response, 1: Idle
};

static struct mgos_modbus* s_modbus = NULL;
static mgos_timer_id req_timer;

static void printBuffer(struct mbuf buffer) {
  char str[2048];
  int length = 0;
  for (int i = 0; i < buffer.len; i++) {
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

  return (lowByte(u16CRC) << 8) | (highByte(u16CRC) & 0xff); //CRC Lower Byte first then Higher Byte
}

static uint8_t verify_crc16(struct mbuf value) {
  value.len -= 2;
  uint16_t u16CRC = calculate_crc16(value);
  if (highByte(u16CRC) != value.buf[value.len] || lowByte(u16CRC) != value.buf[value.len + 1]) { //Since calculate returns lower byte first
    return RESP_INVALID_CRC;
  }
  return RESP_SUCCESS;
}

static void req_timeout_cb(void* arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Request timed out", s_modbus->slave_id_u8, s_modbus->func_code_u8));
  s_modbus->resp_status_u8 = RESP_TIMED_OUT;
  s_modbus->state = 1;
  mgos_uart_set_rx_enabled(s_modbus->uart_no, false);
  s_modbus->cb(RESP_TIMED_OUT, s_modbus->receive_buffer, s_modbus->cb_arg);
  (void)arg;
}

static void uart_cb(int uart_no, void* param) {
  assert(uart_no == s_modbus->uart_no);
  struct mbuf* buffer = &s_modbus->receive_buffer;
  (void)param;

  size_t rx_av = mgos_uart_read_avail(uart_no);
  if (s_modbus->state || rx_av == 0 || rx_av + buffer->len < s_modbus->resp_bytes_u8) {
    return;
  }
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - uart_cb - Receive Buffer: %d, Read Available: %d",
                 s_modbus->slave_id_u8, s_modbus->func_code_u8, s_modbus->receive_buffer.len, rx_av));
  mgos_uart_read_mbuf(uart_no, buffer, rx_av);

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
    LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Invalid Response: %.2x, SlaveID: %.2x, Function: %.2x",
                   s_modbus->slave_id_u8, s_modbus->func_code_u8, s_modbus->resp_status_u8,
                   (uint8_t)buffer->buf[0], (uint8_t)buffer->buf[1]));
    s_modbus->state = 1;
    mgos_clear_timer(req_timer);
    mgos_uart_set_rx_enabled(s_modbus->uart_no, false);
    s_modbus->cb(s_modbus->resp_status_u8, s_modbus->receive_buffer, s_modbus->cb_arg);
    return;
  }
  // evaluate returned Modbus function code
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

  if (buffer->len < s_modbus->resp_bytes_u8) {
    return;
  }

  if (verify_crc16(*buffer) == RESP_INVALID_CRC) {
    LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Invalid CRC", s_modbus->slave_id_u8, s_modbus->func_code_u8));
    s_modbus->resp_status_u8 = RESP_INVALID_CRC;
    s_modbus->state = 1;
    mgos_clear_timer(req_timer);
    mgos_uart_set_rx_enabled(s_modbus->uart_no, false);
    s_modbus->cb(s_modbus->resp_status_u8, s_modbus->receive_buffer, s_modbus->cb_arg);
    return;
  }

  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Modbus response received %d",
                 s_modbus->slave_id_u8, s_modbus->func_code_u8, s_modbus->receive_buffer.len));
  printBuffer(s_modbus->receive_buffer);

  s_modbus->state = 1;
  mgos_clear_timer(req_timer);
  mgos_uart_set_rx_enabled(s_modbus->uart_no, false);
  s_modbus->cb(RESP_SUCCESS, s_modbus->receive_buffer, s_modbus->cb_arg);
}

void init_modbus(uint8_t slave_id, uint8_t func_code, uint8_t total_resp_bytes, mb_response_callback cb, void* cb_arg) {
  s_modbus->cb = cb;
  s_modbus->resp_bytes_u8 = 4;
  s_modbus->slave_id_u8 = slave_id;
  s_modbus->func_code_u8 = func_code;
}

void set_read_address(uint16_t read_address, uint16_t read_qty) {
  s_modbus->read_address_u16 = read_address;
  s_modbus->read_qty_u16 = read_qty;
}

void set_write_address(uint16_t write_address, uint16_t write_qty, uint8_t* data, uint8_t len) {
  s_modbus->write_address_u16 = write_address;
  s_modbus->write_qty_u16 = write_qty;
  if (data != NULL || len > 0) {
    s_modbus->write_data = data;
    s_modbus->write_data_len = len;
  }
}

size_t set_transmit_buffer() {
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
  printBuffer(s_modbus->transmit_buffer);

  return append;
}

bool startTransaction() {
  mbuf_clear(&s_modbus->receive_buffer);
  s_modbus->resp_status_u8 = 0x00;

  if (s_modbus->state && s_modbus->transmit_buffer.len > 0) {
    LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Modbus Transaction Start", s_modbus->slave_id_u8, s_modbus->func_code_u8));
    mgos_uart_set_rx_enabled(s_modbus->uart_no, false);
    mgos_uart_flush(s_modbus->uart_no);
    mgos_msleep(30); //TODO delay for 3.5 Characters length according to baud rate
    req_timer = mgos_set_timer(mgos_sys_config_get_modbus_timeout(), 0, req_timeout_cb, NULL);
    mgos_uart_write(s_modbus->uart_no, s_modbus->transmit_buffer.buf, s_modbus->transmit_buffer.len);
    mgos_uart_set_dispatcher(s_modbus->uart_no, uart_cb, &req_timer);
    s_modbus->state = 0;
    mgos_uart_set_rx_enabled(s_modbus->uart_no, true);
    return true;
  }
  return false;
}

bool readCoils(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Read Coils, Address: %.2x", s_modbus->slave_id_u8, s_modbus->func_code_u8, read_address));
  init_modbus(slave_id, FUNC_READ_COILS, 5, cb, cb_arg);
  set_read_address(read_address, read_qty);

  if (!set_transmit_buffer())
    return false;
  return startTransaction();
}

bool readDiscreteInputs(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Read Discrete Inputs, Address: %.2x", s_modbus->slave_id_u8, s_modbus->func_code_u8, read_address));
  init_modbus(slave_id, FUNC_READ_DISCRETE_INPUTS, 5, cb, cb_arg);
  set_read_address(read_address, read_qty);

  if (!set_transmit_buffer())
    return false;
  return startTransaction();
}

bool readHoldingRegisters(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Read Holding Registers, Address: %.2x", s_modbus->slave_id_u8, s_modbus->func_code_u8, read_address));
  init_modbus(slave_id, FUNC_READ_HOLDING_REGISTERS, 5, cb, cb_arg);
  set_read_address(read_address, read_qty);

  if (!set_transmit_buffer())
    return false;
  return startTransaction();
}

bool readInputRegisters(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Read Input Registers, Address: %.2x", s_modbus->slave_id_u8, s_modbus->func_code_u8, read_address));
  init_modbus(slave_id, FUNC_READ_INPUT_REGISTERS, 5, cb, cb_arg);
  set_read_address(read_address, read_qty);

  if (!set_transmit_buffer())
    return false;
  return startTransaction();
}

bool writeSingleCoil(uint8_t slave_id, uint16_t write_address, uint16_t write_value, mb_response_callback cb, void* cb_arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Write Single Coil, Address: %.2x", s_modbus->slave_id_u8, s_modbus->func_code_u8, write_address));
  init_modbus(slave_id, FUNC_WRITE_SINGLE_COIL, 4, cb, cb_arg);
  set_write_address(write_address, write_value, NULL, 0);

  if (!set_transmit_buffer())
    return false;
  return startTransaction();
}

bool writeSingleRegister(uint8_t slave_id, uint16_t write_address, uint16_t write_value, mb_response_callback cb, void* cb_arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Write Single Register, Address: %.2x", s_modbus->slave_id_u8, s_modbus->func_code_u8, write_address));
  init_modbus(slave_id, FUNC_WRITE_SINGLE_REGISTER, 4, cb, cb_arg);
  set_write_address(write_address, write_value, NULL, 0);

  if (!set_transmit_buffer())
    return false;
  return startTransaction();
}

bool writeMultipleCoils(uint8_t slave_id, uint16_t write_address, uint16_t write_qty,
                        uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Write Multiple Coils, Address: %.2x", s_modbus->slave_id_u8, s_modbus->func_code_u8, write_address));
  init_modbus(slave_id, FUNC_WRITE_MULTIPLE_COILS, 4, cb, cb_arg);
  set_write_address(write_address, write_qty, data, len);

  if (!set_transmit_buffer())
    return false;
  return startTransaction();
}

bool writeMultipleRegisters(uint8_t slave_id, uint16_t write_address, uint16_t write_qty,
                            uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Write Multiple Registers, Address: %.2x", s_modbus->slave_id_u8, s_modbus->func_code_u8, write_address));
  init_modbus(slave_id, FUNC_WRITE_MULTIPLE_REGISTERS, 4, cb, cb_arg);
  set_write_address(write_address, write_qty, data, len);

  if (!set_transmit_buffer())
    return false;
  return startTransaction();
}

bool readWriteMultipleRegisters(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                                uint16_t write_address, uint16_t write_qty,
                                uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Read Write Multiple Registers, Address: %.2x", s_modbus->slave_id_u8, s_modbus->func_code_u8, write_address));
  init_modbus(slave_id, FUNC_READ_WRITE_MULTIPLE_REGISTERS, 4, cb, cb_arg);
  set_read_address(read_address, read_qty);
  set_write_address(write_address, write_qty, data, len);
  if (!set_transmit_buffer())
    return false;
  return startTransaction();
}

bool maskWriteRegister(uint8_t slave_id, uint16_t address, uint16_t and_mask, uint16_t or_mask, mb_response_callback cb, void* cb_arg) {
  LOG(LL_DEBUG, ("SlaveID: %.2x, Function: %.2x - Mask Write Register, Address: %.2x", s_modbus->slave_id_u8, s_modbus->func_code_u8, address));
  init_modbus(slave_id, FUNC_MASK_WRITE_REGISTER, 4, cb, cb_arg);
  s_modbus->read_address_u16 = address;
  s_modbus->mask_and = and_mask;
  s_modbus->mask_or = or_mask;

  if (!set_transmit_buffer())
    return false;
  return startTransaction();
}

bool mgos_modbus_init(void) {
  LOG(LL_DEBUG, ("Initialize modbus"));
  s_modbus = (struct mgos_modbus*)calloc(1, sizeof(*s_modbus));
  if (s_modbus == NULL)
    return false;
  mbuf_init(&s_modbus->transmit_buffer, 256);
  mbuf_init(&s_modbus->receive_buffer, 256);
  s_modbus->state = 1;

  struct mgos_uart_config ucfg;
  mgos_uart_config_set_defaults(mgos_sys_config_get_modbus_uart_no(), &ucfg);
  ucfg.baud_rate = mgos_sys_config_get_modbus_baudrate();
  ucfg.num_data_bits = 8;
  ucfg.parity = mgos_sys_config_get_modbus_parity();
  ucfg.stop_bits = mgos_sys_config_get_modbus_stop_bits();

  s_modbus->uart_no = mgos_sys_config_get_modbus_uart_no();

  struct mgos_uart_dev_config* dcfg = &ucfg.dev;
  dcfg->rx_gpio = mgos_sys_config_get_modbus_uart_rx_pin();
  dcfg->tx_gpio = mgos_sys_config_get_modbus_uart_tx_pin();
  dcfg->hd = mgos_sys_config_get_modbus_tx_en_enable();
  dcfg->tx_en_gpio = mgos_sys_config_get_modbus_tx_en_gpio();
  dcfg->tx_en_gpio_val = mgos_sys_config_get_modbus_tx_en_gpio_val();

  if (!mgos_uart_configure(mgos_sys_config_get_modbus_uart_no(), &ucfg)) {
    return false;
  }
  mgos_uart_set_rx_enabled(mgos_sys_config_get_modbus_uart_no(), false);
  return true;
}
