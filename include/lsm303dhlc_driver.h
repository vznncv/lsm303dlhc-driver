#ifndef LSM303DHLC_DRIVER_H
#define LSM303DHLC_DRIVER_H

#include "lsm303dhlc_driver_base.h"

/**
 * The LSM303DLHC accelerometer driver.
 *
 * It has 2 constructors. One accept I2C pins and frequency.
 * Other accepts I2C interface directly, that allows to share the
 * same interface between several devices/addresses.
 */
class LSM303DLHCAccelerometer : public LSM303DLHCAccelerometerBase {
public:
    /**
     * Constructor
     *
     * @param i2c_ptr I2C interface
     */
    LSM303DLHCAccelerometer(I2C* i2c_ptr);
    /**
     * Constructor
     *
     * @param sda I2C data line pin
     * @param scl I2C clock line pin
     * @param frequency frequency of the I2C interface
     */
    LSM303DLHCAccelerometer(PinName sda, PinName scl, int frequency = 400000);

    virtual uint8_t read_register(uint8_t reg);
    virtual void write_register(uint8_t reg, uint8_t val);
    virtual void read_registers(uint8_t reg, uint8_t* data, uint8_t length);

    ~LSM303DLHCAccelerometer();

private:
    static const int ACC_I2C_ADDRESS = 0x32;

    I2C* i2c_ptr;
    // helper to hold driver state
    uint32_t state;
    static const uint32_t STATE_I2C_CLEAN_UP_BIT = 0x01;
};

#endif // LSM303DHLC_DRIVER_H
