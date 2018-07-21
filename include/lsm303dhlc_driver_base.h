#ifndef LSM303DHLC_DRIVER_BASE_H
#define LSM303DHLC_DRIVER_BASE_H

#include "mbed.h"

/**
 * The base class for LSM303DLHC accelerometer.
 *
 * It implements main accelerometer logic.
 */
class LSM303DLHCAccelerometerBase : NonCopyable<LSM303DLHCAccelerometerBase> {
public:
    LSM303DLHCAccelerometerBase();

    // the methods that should be implemented in the subclass for device register manipulation.
    virtual uint8_t read_register(uint8_t reg) = 0;
    virtual void write_register(uint8_t reg, uint8_t val) = 0;
    virtual void read_registers(uint8_t reg, uint8_t* data, uint8_t length) = 0;

    // helper methods to set/reset separate bit of the device register
    /**
     * Update specified register.
     *
     * Only bits that are selected by mask will be updated.
     *
     * @param reg register address
     * @param val value to set
     * @param mask value mask
     */
    virtual void update_register(uint8_t reg, uint8_t val, uint8_t mask);

    /**
     * Version of the read_register method with mask.
     *
     * Any bit in the result that corresponds zero bits in the mask will be set to 0.
     *
     * @param reg register address
     * @param mask mask
     * @return masked register value
     */
    virtual uint8_t read_register(uint8_t reg, uint8_t mask);

    /**
     * Initialize device with default settings and test connection.
     *
     * @return 0, if device is initialize correctly, otherwise 1.
     */
    int init();

    /**
     * Accelerometer register addresses
     */
    enum Registers {
        WHO_AM_I_ADDR = 0x0F, /* Device identification register */
        CTRL_REG1_A = 0x20, /* Control register 1 acceleration */
        CTRL_REG2_A = 0x21, /* Control register 2 acceleration */
        CTRL_REG3_A = 0x22, /* Control register 3 acceleration */
        CTRL_REG4_A = 0x23, /* Control register 4 acceleration */
        CTRL_REG5_A = 0x24, /* Control register 5 acceleration */
        CTRL_REG6_A = 0x25, /* Control register 6 acceleration */
        REFERENCE_A = 0x26, /* Reference register acceleration */
        STATUS_REG_A = 0x27, /* Status register acceleration */
        OUT_X_L_A = 0x28, /* Output Register X acceleration */
        OUT_X_H_A = 0x29, /* Output Register X acceleration */
        OUT_Y_L_A = 0x2A, /* Output Register Y acceleration */
        OUT_Y_H_A = 0x2B, /* Output Register Y acceleration */
        OUT_Z_L_A = 0x2C, /* Output Register Z acceleration */
        OUT_Z_H_A = 0x2D, /* Output Register Z acceleration */
        FIFO_CTRL_REG_A = 0x2E, /* FIFO control Register acceleration */
        FIFO_SRC_REG_A = 0x2F, /* FIFO src Register acceleration */

        INT1_CFG_A = 0x30, /* Interrupt 1 configuration Register acceleration */
        INT1_SOURCE_A = 0x31, /* Interrupt 1 source Register acceleration */
        INT1_THS_A = 0x32, /* Interrupt 1 Threshold register acceleration */
        INT1_DURATION_A = 0x33, /* Interrupt 1 DURATION register acceleration */

        INT2_CFG_A = 0x34, /* Interrupt 2 configuration Register acceleration */
        INT2_SOURCE_A = 0x35, /* Interrupt 2 source Register acceleration */
        INT2_THS_A = 0x36, /* Interrupt 2 Threshold register acceleration */
        INT2_DURATION_A = 0x37, /* Interrupt 2 DURATION register acceleration */

        CLICK_CFG_A = 0x38, /* Click configuration Register acceleration */
        CLICK_SOURCE_A = 0x39, /* Click 2 source Register acceleration */
        CLICK_THS_A = 0x3A, /* Click 2 Threshold register acceleration */

        TIME_LIMIT_A = 0x3B, /* Time Limit Register acceleration */
        TIME_LATENCY_A = 0x3C, /* Time Latency Register acceleration */
        TIME_WINDOW_A = 0x3D, /* Time window register acceleration */
    };

    enum PowerMode {
        NORMAL_POWER_MODE = 0,
        LOW_POWER_MODE = 1
    };

    /**
     * Set power mode.
     *
     * It affects on allowed output data rates.
     *
     * @note
     * Set power mode before output data range, as it can change it.
     *
     * @param power_mode
     */
    void set_power_mode(PowerMode power_mode);

    /**
     * Get current power mode.
     *
     * @return
     */
    PowerMode get_power_mode();

    enum OutputDataRate {
        ODR_NONE = 0x03, // the sensor is disabled
        ODR_1HZ = 0x13,
        ODR_10HZ = 0x23,
        ODR_25HZ = 0x33,
        ODR_50HZ = 0x43,
        ODR_100HZ = 0x53,
        ODR_200HZ = 0x63,
        ODR_400HZ = 0x73,
        ODR_1620HZ = 0x82, // low power mode only
        ODR_1344HZ = 0x91, // normal mode only
        ODR_5376HZ = 0x92, // low power mode only
    };

    /**
     * Set output data rate.
     *
     * @param odr
     */
    void set_output_data_rate(OutputDataRate odr);

    /**
     * Get current output data rate.
     *
     * @return
     */
    OutputDataRate get_output_data_rate();

    /**
     * Get current output data rate in Hz.
     *
     * @return
     */
    float get_output_data_rate_hz();

    enum FullScale {
        FULL_SCALE_2G = 0x00,
        FULL_SCALE_4G = 0x10,
        FULL_SCALE_8G = 0x20,
        FULL_SCALE_16G = 0x30,
    };

    /**
     * Get full scale, i.e. maximum amplitude that accelerometer can detect.
     *
     * @note
     * Full scale influences on "Unit/LSB" parameter.
     *
     * @param fs
     */
    void set_full_scale(FullScale fs);

    /**
     * Get current full scale.
     *
     * @return
     */
    FullScale get_full_scale();

    static const float GRAVITY_OF_EARTH = 9.80665f;

    /**
     * Get value of the (m/s^2)/LSB.
     *
     * @return
     */
    float get_unit_per_lsb();

    enum HighPassFilterMode {
        HPF_OFF = 0xFF, /* Switch off filter */
        HPF_CF0 = 0x00, /* Set cutoff 0 */
        HPF_CF1 = 0x10, /* Set cutoff 1 */
        HPF_CF2 = 0x20, /* Set cutoff 2 */
        HPF_CF3 = 0x30, /* Set cutoff 3 */
    };

    /**
     * Set high pass filter mode.
     *
     * Cut off calculation. Let's denote the filter coefficient \f$HP_C\$f. The \f$HP_C=0\$f if hpf is HPF_CF0,
     * \f$HP_C=1\$f - HPF_CF1, \f$HP_C=2\$f - HPF_CF2, \f$HP_C=3\$f - HPF_CF3.
     * Then cut off frequency can be calculated using formula: \f$f_{cut off} = -ln(1 - \fraq{3}{25 2^{HP_C}}) \fraq{f_s}{2 \pi}\f$.
     * The following approximation can be used: \f$f_{cut off} = \fraq{f_s}{50 2^{HP_C}}\f$.
     *
     * For example if \f$HP_C=1\$f and \f$f_s=400$f, then \f$f_{cut off} = 4\f$.
     *
     * @param hpf
     */
    void set_high_pass_filter_mode(HighPassFilterMode hpf);

    /**
     * Get high pass filter mode.
     *
     * @return
     */
    HighPassFilterMode get_high_pass_filter_mode();

    /**
     * Calculate current cut off frequency of the high pass filter.
     *
     * The frequency depends on output data rate and high pass filter mode.
     *
     * @return
     */
    float get_high_pass_filter_cut_off_frequency();

    // TODO: add methods for FIFO control

    enum DatadaReadyInterruptMode {
        DRDY_ENABLE = 1,
        DRDY_DISABLE = 0
    };

    /**
     * Enable/disable accelerometer data ready interrupt on pin INT1.
     *
     * @param drdy_mode
     */
    void set_data_ready_interrupt_mode(DatadaReadyInterruptMode drdy_mode);

    /**
     * Check if data ready interrupt is disabled/enabled.
     *
     * @return
     */
    DatadaReadyInterruptMode get_data_ready_interrupt_mode();

    enum HighResolutionOutputMode {
        HRO_ENABLED = 1,
        HRO_DISABLED = 0
    };

    /**
     * Enable/disable high resolution output mode.
     *
     * If high resolution mode is enable, output data resolution is 12 bit.
     * If high resolution mode is disabled, output data resolution is 10 bit. In this case
     * the output data of the method read_data() has the same format, but last 2 bits will be zero.
     *
     * @param hr
     */
    void set_high_resolution_output_mode(HighResolutionOutputMode hro);

    /**
     * Check if high resolution output mode is disabled/enabled.
     *
     * @return
     */
    HighResolutionOutputMode get_high_resolution_output_mode();

    /**
     * Read current accelerometer data.
     *
     * The data will be stored in the array with order: x_a, y_a, z_a.
     * The values is converted into m/s^2 units.
     *
     * @note the data can have offset
     *
     * @param data
     */
    void read_data(float data[3]);

    /**
     * Read raw accelerometer data.
     *
     * The data will be stored in the array with order: x_a, y_a, z_a.
     * The values represent signed integers. To get m/s^2 units, the values should be multiply
     * by the value that is returned by method LSM303DLHCAccelerometerBase::get_unit_per_lsb.
     *
     * @param data
     */
    void read_data_16(int16_t data[3]);

protected:
    virtual ~LSM303DLHCAccelerometerBase();

private:
    // TODO: check different mems to be sure that value of the "WHO_AM_I_ADDR" register is stable.
    static const int DEVICE_ID = 0x33;

    // current unit/lsb
    float unit_per_lsb;
};

#endif // LSM303DHLC_DRIVER_BASE_H
