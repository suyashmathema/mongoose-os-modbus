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

#ifndef CS_MOS_LIBS_MODBUS_INCLUDE_MGOS_MODBUS_H_
#define CS_MOS_LIBS_MODBUS_INCLUDE_MGOS_MODBUS_H_

#include "mgos.h"

#if defined(__cplusplus)
extern "C" {  // Make sure we have C-declarations in C++ programs
#endif

// Modbus exception codes
/**
    Modbus protocol illegal function exception.
    
    The function code received in the query is not an allowable action for
    the server (or slave). This may be because the function code is only
    applicable to newer devices, and was not implemented in the unit
    selected. It could also indicate that the server (or slave) is in the
    wrong state to process a request of this type, for example because it is
    unconfigured and is being asked to return register values.
    
    @ingroup constant
    */
static const uint8_t EX_ILLEGAL_FUNCTION = 0x01;

/**
    Modbus protocol illegal data address exception.
    
    The data address received in the query is not an allowable address for 
    the server (or slave). More specifically, the combination of reference 
    number and transfer length is invalid. For a controller with 100 
    registers, the ADU addresses the first register as 0, and the last one 
    as 99. If a request is submitted with a starting register address of 96 
    and a quantity of registers of 4, then this request will successfully 
    operate (address-wise at least) on registers 96, 97, 98, 99. If a 
    request is submitted with a starting register address of 96 and a 
    quantity of registers of 5, then this request will fail with Exception 
    Code 0x02 "Illegal Data Address" since it attempts to operate on 
    registers 96, 97, 98, 99 and 100, and there is no register with address 
    100. 
    
    @ingroup constant
    */
static const uint8_t EX_ILLEGAL_DATA_ADDRESS = 0x02;

/**
    Modbus protocol illegal data value exception.
    
    A value contained in the query data field is not an allowable value for 
    server (or slave). This indicates a fault in the structure of the 
    remainder of a complex request, such as that the implied length is 
    incorrect. It specifically does NOT mean that a data item submitted for 
    storage in a register has a value outside the expectation of the 
    application program, since the MODBUS protocol is unaware of the 
    significance of any particular value of any particular register.
    
    @ingroup constant
    */
static const uint8_t EX_ILLEGAL_DATA_VALUE = 0x03;

/**
    Modbus protocol slave device failure exception.
    
    An unrecoverable error occurred while the server (or slave) was
    attempting to perform the requested action.
    
    @ingroup constant
    */
static const uint8_t EX_SLAVE_DEVICE_FAILURE = 0x04;

// Class-defined success/exception codes
/**
    ModbusMaster success.
    
    Modbus transaction was successful; the following checks were valid:
      - slave ID
      - function code
      - response code
      - data
      - CRC
      
    @ingroup constant
    */
static const uint8_t RESP_SUCCESS = 0x00;

/**
    ModbusMaster invalid response slave ID exception.
    
    The slave ID in the response does not match that of the request.
    
    @ingroup constant
    */
static const uint8_t RESP_INVALID_SLAVE_ID = 0xE0;

/**
    ModbusMaster invalid response function exception.
    
    The function code in the response does not match that of the request.
    
    @ingroup constant
    */
static const uint8_t RESP_INVALID_FUNCTION = 0xE1;

/**
    ModbusMaster response timed out exception.
    
    The entire response was not received within the timeout period, 
    ModbusMaster::ku8MBResponseTimeout. 
    
    @ingroup constant
    */
static const uint8_t RESP_TIMED_OUT = 0xE2;

/**
    ModbusMaster invalid response CRC exception.
    
    The CRC in the response does not match the one calculated.
    
    @ingroup constant
    */
static const uint8_t RESP_INVALID_CRC = 0xE3;

// Modbus function codes for bit access
static const uint8_t FUNC_READ_COILS = 0x01;            ///< Modbus function 0x01 Read Coils
static const uint8_t FUNC_READ_DISCRETE_INPUTS = 0x02;  ///< Modbus function 0x02 Read Discrete Inputs
static const uint8_t FUNC_WRITE_SINGLE_COIL = 0x05;     ///< Modbus function 0x05 Write Single Coil
static const uint8_t FUNC_WRITE_MULTIPLE_COILS = 0x0F;  ///< Modbus function 0x0F Write Multiple Coils

// Modbus function codes for 16 bit access
static const uint8_t FUNC_READ_HOLDING_REGISTERS = 0x03;         ///< Modbus function 0x03 Read Holding Registers
static const uint8_t FUNC_READ_INPUT_REGISTERS = 0x04;           ///< Modbus function 0x04 Read Input Registers
static const uint8_t FUNC_WRITE_SINGLE_REGISTER = 0x06;          ///< Modbus function 0x06 Write Single Register
static const uint8_t FUNC_WRITE_MULTIPLE_REGISTERS = 0x10;       ///< Modbus function 0x10 Write Multiple Registers
static const uint8_t FUNC_MASK_WRITE_REGISTER = 0x16;            ///< Modbus function 0x16 Mask Write Register
static const uint8_t FUNC_READ_WRITE_MULTIPLE_REGISTERS = 0x17;  ///< Modbus function 0x17 Read Write Multiple Registers

struct mb_request_info {
    uint8_t slave_id;
    uint16_t read_address;
    uint16_t read_qty;
    uint16_t write_address;
    uint16_t write_qty;
    uint8_t mask_and;
    uint8_t mask_or;
    uint8_t func_code;
};

/* Modbus response callback */
typedef void (*mb_response_callback)(uint8_t status, struct mb_request_info info, struct mbuf response, void* param);

bool mb_read_coils(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                   mb_response_callback cb, void* cb_arg);

bool mb_read_discrete_inputs(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                             mb_response_callback cb, void* cb_arg);

bool mb_read_holding_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                               mb_response_callback cb, void* cb_arg);

bool mb_read_input_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                             mb_response_callback cb, void* cb_arg);

bool mb_write_single_coil(uint8_t slave_id, uint16_t write_address, uint16_t write_value,
                          mb_response_callback cb, void* cb_arg);

bool mb_write_single_register(uint8_t slave_id, uint16_t write_address, uint16_t write_value,
                              mb_response_callback cb, void* cb_arg);

bool mb_write_multiple_coils(uint8_t slave_id, uint16_t write_address, uint16_t write_qty,
                             uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg);

bool mb_write_multiple_registers(uint8_t slave_id, uint16_t write_address, uint16_t write_qty,
                                 uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg);

bool mb_read_write_multiple_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                                      uint16_t write_address, uint16_t write_qty, uint8_t* data,
                                      uint8_t len, mb_response_callback cb, void* cb_arg);

bool mb_mask_write_register(uint8_t slave_id, uint16_t address, uint16_t andMask, uint16_t orMask,
                            mb_response_callback cb, void* cb_arg);

long parse_value_long_inverse_32(uint8_t* value);

float parse_value_float32(uint8_t* value);

char* mb_map_response(const char* json_map, struct mbuf* mb_resp, struct mb_request_info* info);

char* mb_map_responsef(const char* json_file, struct mbuf* mb_resp, struct mb_request_info* info);

bool mgos_modbus_connect();

#ifdef __cplusplus
}
#endif

#endif /* CS_MOS_LIBS_MODBUS_INCLUDE_MGOS_MODBUS_H_ */
