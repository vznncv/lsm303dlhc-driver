#ifndef LSM303DLHC_MAGNETOMETER_DRIVER_H
#define LSM303DLHC_MAGNETOMETER_DRIVER_H

#include "lsm303dlhc_utils.h"
#include "mbed.h"

namespace lsm303dlhc {

/**
 * The LSM303DLHC magnetometer driver.
 *
 * @note
 * The magnetometer has DRDY pin that have high level if new data is available.
 * It cannot be disabled or somehow configured through I2C.
 */
class LSM303DLHCMagnetometer : NonCopyable<LSM303DLHCMagnetometer> {
public:
    /**
     * Constructor
     *
     * @param i2c_ptr I2C interface
     */
    LSM303DLHCMagnetometer(I2C *i2c_ptr);

    /**
     * Constructor.
     *
     * @param sda I2C data line pin
     * @param scl I2C clock line pin
     * @param frequency I2C bus frequency
     */
    LSM303DLHCMagnetometer(PinName sda, PinName scl, int frequency = 400000);

    virtual ~LSM303DLHCMagnetometer();

    /**
     * Initialize device with default settings and test connection.
     *
     * Note: this method is idempotent.
     *
     * @param start if it's `true`, then initially sensor will be enabled, otherwise disabled.
     * @return 0, if device is initialize correctly, otherwise non-zero error code.
     */
    int init(bool start = true);

    /**
     * Magnetometer register addresses
     */
    enum Registers {
        WHO_AM_I_ADDR = 0x0F, // Device identification register
        CRA_REG_M = 0x00, // Control register A magnetic field
        CRB_REG_M = 0x01, // Control register B magnetic field
        MR_REG_M = 0x02, // Control register MR magnetic field
        OUT_X_H_M = 0x03, // Output Register X magnetic field
        OUT_X_L_M = 0x04, // Output Register X magnetic field
        OUT_Z_H_M = 0x05, // Output Register Z magnetic field
        OUT_Z_L_M = 0x06, // Output Register Z magnetic field
        OUT_Y_H_M = 0x07, // Output Register Y magnetic field
        OUT_Y_L_M = 0x08, // Output Register Y magnetic field
        SR_REG_M = 0x09, // Status Register magnetic field

        IRA_REG_M = 0x0A, // IRA Register magnetic field
        IRB_REG_M = 0x0B, // IRB Register magnetic field
        IRC_REG_M = 0x0C, // IRC Register magnetic field

        TEMP_OUT_H_M = 0x31, // Temperature Register magnetic field
        TEMP_OUT_L_M = 0x32, // Temperature Register magnetic field
    };

    /**
     * Read accelerometer register.
     *
     * @param reg register address
     * @return register value
     */
    uint8_t read_register(uint8_t reg);

    /**
     * Write value to accelerometer register.
     *
     * @param reg register address
     * @param val register value
     */
    void write_register(uint8_t reg, uint8_t val);

    enum TemperatureSensorMode {
        TS_ENABLE = 0x80,
        TS_DISABLE = 0x00
    };

    /**
     * Disable/enable temperature sensor.
     *
     * @param tsm
     */
    void set_temperature_sensor_mode(TemperatureSensorMode tsm);

    /**
     * Check if temperature sensor is disabled/enabled.
     *
     * @return zero value is sensor id disabled, otherwise non-zero value.
     */
    TemperatureSensorMode get_temperature_sensor_mode();

    /**
     * Get current temperature.
     *
     * @todo
     * Check sensitivity and offset of the sensor.
     *
     * @return
     */
    float read_temperature();

    /**
     * Get raw data from temperature sensor.
     *
     * The value in the celsius can be get using the formula: t_c = val * sensitivity + zero_offset.
     *
     * @return
     */
    int16_t read_temperature_16();

    /**
     * Get temperature sensor sensitivity.
     *
     * @return
     */
    float get_temperature_sensor_sensitivity();

    /**
     * Get temperature sensor offset.
     *
     * @return
     */
    float get_temperature_sensor_zero_offset();

    enum OutputDataRate {
        ODR_0_75_HZ = 0x00,
        ODR_1_5_HZ = 0x01,
        ODR_3_0_HZ = 0x02,
        ODR_7_5_HZ = 0x03,
        ODR_15_HZ = 0x04,
        ODR_30_HZ = 0x05,
        ODR_75_HZ = 0x06,
        ODR_220_HZ = 0x07,
    };

    /**
     * Set output data rate.
     *
     * @param odr
     */
    void set_output_data_rate(OutputDataRate odr);

    /**
     * Get output data rate.
     *
     * @return
     */
    OutputDataRate get_output_data_rate();

    /**
     * Get output data rate in HZ.
     *
     * @return
     */
    float get_output_data_rate_hz();

    enum MagnetometerMode {
        M_DISABLE = 0,
        M_ENABLE = 1
    };

    /**
     * Disable/enable magnetometer.
     *
     * @param mm
     */
    void set_magnetometer_mode(MagnetometerMode mm);

    /**
     * Check if magnetometer is enabled/disabled.
     *
     * @return @c M_ENABLE if magnetometer is enabled, else @c M_DISABLE
     */
    MagnetometerMode get_magnetometer_mode();

    enum FullScale {
        FULL_SCALE_1_3_G = 0x20, // Full scale = ±1.3 Gauss
        FULL_SCALE_1_9_G = 0x40, // Full scale = ±1.9 Gauss
        FULL_SCALE_2_5_G = 0x60, // Full scale = ±2.5 Gauss
        FULL_SCALE_4_0_G = 0x80, // Full scale = ±4.0 Gauss
        FULL_SCALE_4_7_GA = 0xA0, // Full scale = ±4.7 Gauss
        FULL_SCALE_5_6_G = 0xC0, // Full scale = ±5.6 Gauss
        FULL_SCALE_8_1_G = 0xE0 // Full scale = ±8.1 Gauss
    };

    /**
     * Set full scale, i.e. maximum amplitude that magnetometer can detect.
     *
     * @param fs
     */
    void set_full_scale(FullScale fs);

    /**
     * Get full scale.
     *
     * @return
     */
    FullScale get_full_scale();

    /**
     * Get magnetometer sensitivity.
     *
     * @note
     * Z axis sensitivity differs from X and Y
     *
     * @param axis_no axis (0 - X, 1 - Y, 2 - Z)
     * @return
     */
    float get_sensitivity(int axis_no);

    /**
     * Read current accelerometer data.
     *
     * The data will be placed into \p data array in order: x, y, z.
     * The values is converted into gauss units.
     *
     * @note
     * The data can have offset.
     *
     * @todo
     * Add possibility to add user offset for a calibration.
     *
     * @param data
     */
    void read_data(float data[3]);

    /**
     * Read raw accelerometer data.
     *
     * The data will be placed into \p data array in order: x, y, z.
     * The values represent signed integers. To get gauss units, the values should be multiply
     * by the value that is returned by method LSM303DHLCMagnetometer::get_sensitivity.
     *
     * @param data
     */
    void read_data_16(int16_t data[3]);

private:
    I2CDevice _i2c_device;

    static const uint8_t _I2C_ADDRESS = 0x3C;

    // TODO: check different mems to be sure that value of the "WHO_AM_I_ADDR" register is stable.
    static const int _DEVICE_ID = 0x3C;

    static const float _temperature_sensitivity;
    static const float _temperature_offset;

    // IRx_REG_M register values
    static const uint8_t _IRA_REG_M_VAL = 0x48;
    static const uint8_t _IRB_REG_M_VAL = 0x34;
    static const uint8_t _IRC_REG_M_VAL = 0x33;

    float _xy_mag_sensitivity;
    float _z_mag_sensitivity;
    // Sometime after first read in the continuous mode magnetometer hangs.
    // To fix it, we need to enable continuous mode again.
    int8_t _mode_state;
};
}

#endif // LSM303DLHC_MAGNETOMETER_DRIVER_H
