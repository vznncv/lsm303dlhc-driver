#include "lsm303dhlc_driver.h"
#include "mbed_error.h"

LSM303DLHCAccelerometer::LSM303DLHCAccelerometer(mbed::I2C* i2c)
    : i2c_ptr(i2c)
    , state(0)
{
}

LSM303DLHCAccelerometer::LSM303DLHCAccelerometer(PinName sda, PinName scl, int frequency)
    : state(0)
{
    i2c_ptr = new I2C(sda, scl);
    i2c_ptr->frequency(frequency);
    state |= STATE_I2C_CLEAN_UP_BIT;
}

LSM303DLHCAccelerometer::~LSM303DLHCAccelerometer()
{
    if (state & STATE_I2C_CLEAN_UP_BIT) {
        delete i2c_ptr;
    }
}

uint8_t LSM303DLHCAccelerometer::read_register(uint8_t reg)
{
    uint8_t val;
    int res;

    // write register address
    res = i2c_ptr->write(ACC_I2C_ADDRESS, (char*)&reg, 1, true);
    if (res) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "register reading failed");
    }
    // get register value
    res = i2c_ptr->read(ACC_I2C_ADDRESS, (char*)&val, 1);
    if (res) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "register reading failed");
    }
    return val;
}

void LSM303DLHCAccelerometer::write_register(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { reg, val };
    // write register address and value
    int res = i2c_ptr->write(ACC_I2C_ADDRESS, (char*)data, 2);
    if (res) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "register writing failed");
    }
}

void LSM303DLHCAccelerometer::read_registers(uint8_t reg, uint8_t* data, uint8_t length)
{
    int res;

    // read multiple registers
    reg |= 0x80;

    // write register address
    res = i2c_ptr->write(ACC_I2C_ADDRESS, (char*)&reg, 1, true);
    if (res) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "registers reading failed");
    }
    // get register value
    res = i2c_ptr->read(ACC_I2C_ADDRESS, (char*)data, length);
    if (res) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "registers reading failed");
    }
}
