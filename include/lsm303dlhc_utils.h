#ifndef LSM303DLHC_UTILS_H
#define LSM303DLHC_UTILS_H

#include "mbed.h"

namespace lsm303dlhc {

/**
 * Inner LSM303DLHC driver interface.
 *
 * It shouldn't be used directly.
 */
class I2CDevice : public NonCopyable<I2CDevice> {
public:
    /**
     * Constructor
     *
     * @param address device address on I2C bus
     * @param i2c_ptr I2C interface
     */
    I2CDevice(uint8_t address, I2C* i2c_ptr);

    /**
     * Constructor.
     *
     * @param address address device address on I2C bus
     * @param sda I2C data line pin
     * @param scl I2C clock line pin
     * @param frequency I2C bus frequency
     */
    I2CDevice(uint8_t address, PinName sda, PinName scl, int frequency = 400000);

    virtual ~I2CDevice();

    /**
     * Read device register.
     *
     * @param reg register address
     * @return register value
     */
    uint8_t read_register(uint8_t reg);

    /**
     * Write value to register.
     *
     * @param reg register address
     * @param val register value
     */
    void write_register(uint8_t reg, uint8_t val);

    /**
     * Update specified register.
     *
     * Only bits that are selected by mask will be updated.
     *
     * @param reg register address
     * @param val value to set
     * @param mask value mask
     */
    void update_register(uint8_t reg, uint8_t val, uint8_t mask);

    /**
     * Version of the read_register method with mask.
     *
     * Any bit in the result that corresponds zero bits in the mask will be set to 0.
     *
     * @param reg register address
     * @param mask mask
     * @return masked register value
     */
    uint8_t read_register(uint8_t reg, uint8_t mask);

    /**
     * Read several registers, starting with address \p reg.
     * This method cannot be invoked in the ISR context.
     *
     * @param reg
     * @param data
     * @param length
     */
    void read_registers(uint8_t reg, uint8_t* data, uint8_t length);

    // the asynchronous reading isn't implemented as I2C::transfer isn't interrupt safe
    // (probably it should use CriticalSectionLock instead of PlatformMutex)
    // int read_registers_async(uint8_t reg, uint8_t* data, uint8_t length, const Callback<void(void)>& callback);
    // int read_registers_async(uint8_t reg, uint8_t* data, uint8_t length, const Callback<void(uint8_t* data, uint8_t length)>& callback);

private:
    uint8_t address;
    // helper variable with state flags
    uint8_t state;
    enum StateFlags {
        CleanupI2C = 0x01
    };

    I2C* i2c_ptr;
};
}
#endif // LSM303DLHC_UTILS_H
