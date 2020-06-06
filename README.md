## Overview
This is a Modbus master library for mongoose os. 

## Features
The following Modbus functions are available:

Discrete Coils/Flags

  - 0x01 - Read Coils
  - 0x02 - Read Discrete Inputs
  - 0x05 - Write Single Coil
  - 0x0F - Write Multiple Coils

Registers

  - 0x03 - Read Holding Registers
  - 0x04 - Read Input Registers
  - 0x06 - Write Single Register
  - 0x10 - Write Multiple Registers
  - 0x16 - Mask Write Register
  - 0x17 - Read Write Multiple Registers

## Hardware
Tested on ESP32 and mongoose os version 2.17.0


## Example
You can check [Mongoose OS Modbus Example](https://github.com/suyashmathema/mongoose-os-modbus-example) for a sample application.

_Project inspired by [ModbusMaster](https://github.com/4-20ma/ModbusMaster)._
