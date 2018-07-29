#include "lsm303dlhc_utils.h"

using namespace lsm303dlhc;

I2CDevice::I2CDevice(uint8_t address, I2C* i2c_ptr)
{
    this->address = address;
    this->i2c_ptr = i2c_ptr;
    this->state = 0x00;
}

I2CDevice::I2CDevice(uint8_t address, PinName sda, PinName scl, int frequency)
{
    this->address = address;
    this->i2c_ptr = new I2C(sda, scl);
    this->i2c_ptr->frequency(frequency);
    this->state = 0x00 | CleanupI2C;
}

I2CDevice::~I2CDevice()
{
    if (this->state && CleanupI2C) {
        delete i2c_ptr;
    }
}

uint8_t I2CDevice::read_register(uint8_t reg)
{
    uint8_t val;
    int res;

    // write register address
    res = i2c_ptr->write(address, (char*)&reg, 1, true);
    if (res) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "register reading failed");
    }
    // get register value
    res = i2c_ptr->read(address, (char*)&val, 1);
    if (res) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_READ_FAILED), "register reading failed");
    }
    return val;
}

void I2CDevice::write_register(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { reg, val };
    // write register address and value
    int res = i2c_ptr->write(address, (char*)data, 2);
    if (res) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "register writing failed");
    }
}

void I2CDevice::update_register(uint8_t reg, uint8_t val, uint8_t mask)
{
    uint8_t reg_val = read_register(reg);
    reg_val &= ~mask;
    val = val & mask;
    reg_val |= val;
    write_register(reg, reg_val);
}

uint8_t I2CDevice::read_register(uint8_t reg, uint8_t mask)
{
    return read_register(reg) & mask;
}

void I2CDevice::read_registers(uint8_t reg, uint8_t* data, uint8_t length)
{
    int res;

    // write register address
    res = i2c_ptr->write(address, (char*)&reg, 1, true);
    if (res) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "registers reading failed");
    }
    // get register value
    res = i2c_ptr->read(address, (char*)data, length);
    if (res) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_READ_FAILED), "registers reading failed");
    }
}
