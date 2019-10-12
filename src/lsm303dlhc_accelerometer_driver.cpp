#include "lsm303dlhc_accelerometer_driver.h"
#include "math.h"
#include "mbed_error.h"

using namespace lsm303dlhc;

LSM303DLHCAccelerometer::LSM303DLHCAccelerometer(I2C *i2c_ptr)
    : _i2c_device(_I2C_ADDRESS, i2c_ptr)
    , _sensitivity(0)
{
}

LSM303DLHCAccelerometer::LSM303DLHCAccelerometer(PinName sda, PinName scl, int frequency)
    : _i2c_device(_I2C_ADDRESS, sda, scl, frequency)
    , _sensitivity(0)
{
}

LSM303DLHCAccelerometer::~LSM303DLHCAccelerometer()
{
}

int LSM303DLHCAccelerometer::init(bool start)
{
    // check device id
    int device_id = _i2c_device.read_register(WHO_AM_I_ADDR);
    if (device_id != _DEVICE_ID) {
        return MBED_ERROR_CODE_INITIALIZATION_FAILED;
    }

    // set default modes
    _reboot_memory_content();
    set_data_ready_interrupt_mode(DRDY_DISABLE);
    set_fifo_mode(FIFO_DISABLE);
    set_fifo_watermark(0);
    set_full_scale(FULL_SCALE_2G);
    set_high_pass_filter_mode(HPF_OFF);
    set_high_resolution_output_mode(HRO_ENABLED);
    set_power_mode(NORMAL_POWER_MODE);
    _clear_data();

    LSM303DLHCAccelerometer::OutputDataRate expected_odr = start ? ODR_25HZ : ODR_NONE;
    set_output_data_rate(expected_odr);
    // check that ODR is set correctly
    if (get_output_data_rate() != expected_odr) {
        return MBED_ERROR_CODE_INITIALIZATION_FAILED;
    }

    return MBED_SUCCESS;
}

uint8_t LSM303DLHCAccelerometer::read_register(uint8_t reg)
{
    return _i2c_device.read_register(reg);
}

void LSM303DLHCAccelerometer::write_register(uint8_t reg, uint8_t val)
{
    _i2c_device.write_register(reg, val);
}

void LSM303DLHCAccelerometer::set_power_mode(PowerMode power_mode)
{
    // update power mode bit
    _i2c_device.update_register(CTRL_REG1_A, (uint8_t)(power_mode << 3), 0x08);
}

LSM303DLHCAccelerometer::PowerMode LSM303DLHCAccelerometer::get_power_mode()
{
    uint8_t val = _i2c_device.read_register(CTRL_REG1_A, 0x08);
    return val ? LOW_POWER_MODE : NORMAL_POWER_MODE;
}

void LSM303DLHCAccelerometer::set_output_data_rate(OutputDataRate odr)
{
    OutputDataRate prev_odr = get_output_data_rate();
    if (prev_odr == odr) {
        return;
    }

    if (odr == ODR_NONE) {
        // set power down mode
        _i2c_device.update_register(CTRL_REG1_A, 0x00, 0xF0);
    } else {
        PowerMode power_mode = get_power_mode();

        switch (power_mode) {
        case NORMAL_POWER_MODE:
            if (!(odr & 0x01)) {
                MBED_ERROR(MBED_ERROR_CONFIG_MISMATCH, "Invalid ODR for normal power mode");
            }
            break;
        case LOW_POWER_MODE:
            if (!(odr & 0x02)) {
                MBED_ERROR(MBED_ERROR_CONFIG_MISMATCH, "Invalid ODR for low power mode");
            }
            break;
        }

        if (prev_odr == ODR_NONE) {
            _clear_data();
        }

        // set ODR and enable axes
        _i2c_device.update_register(CTRL_REG1_A, (odr & 0xF0) | 0x07, 0xF7);
    }
}

LSM303DLHCAccelerometer::OutputDataRate LSM303DLHCAccelerometer::get_output_data_rate()
{
    uint8_t val = _i2c_device.read_register(CTRL_REG1_A, 0xF0);
    PowerMode power_mode;
    OutputDataRate odr;

    switch (val) {
    case 0x00:
        odr = ODR_NONE;
        break;
    case 0x10:
        odr = ODR_1HZ;
        break;
    case 0x20:
        odr = ODR_10HZ;
        break;
    case 0x30:
        odr = ODR_25HZ;
        break;
    case 0x40:
        odr = ODR_50HZ;
        break;
    case 0x50:
        odr = ODR_100HZ;
        break;
    case 0x60:
        odr = ODR_200HZ;
        break;
    case 0x70:
        odr = ODR_400HZ;
        break;
    case 0x80:
        odr = ODR_1620HZ;
        break;
    case 0x90:
        power_mode = get_power_mode();
        odr = power_mode == NORMAL_POWER_MODE ? ODR_1344HZ : ODR_5376HZ;
    default:
        MBED_ERROR(MBED_ERROR_INVALID_DATA_DETECTED, "Invalid CTRL_REG1_A value");
    }
    return odr;
}

float LSM303DLHCAccelerometer::get_output_data_rate_hz()
{
    float f_odr;

    switch (get_output_data_rate()) {
    case LSM303DLHCAccelerometer::ODR_NONE:
        f_odr = 0;
        break;
    case LSM303DLHCAccelerometer::ODR_1HZ:
        f_odr = 1.0f;
        break;
    case LSM303DLHCAccelerometer::ODR_10HZ:
        f_odr = 10.0f;
        break;
    case LSM303DLHCAccelerometer::ODR_25HZ:
        f_odr = 25.0f;
        break;
    case LSM303DLHCAccelerometer::ODR_50HZ:
        f_odr = 50.0f;
        break;
    case LSM303DLHCAccelerometer::ODR_100HZ:
        f_odr = 100.0f;
        break;
    case LSM303DLHCAccelerometer::ODR_200HZ:
        f_odr = 200.0f;
        break;
    case LSM303DLHCAccelerometer::ODR_400HZ:
        f_odr = 400.0f;
        break;
    case LSM303DLHCAccelerometer::ODR_1620HZ:
        f_odr = 1620.0f;
        break;
    case LSM303DLHCAccelerometer::ODR_1344HZ:
        f_odr = 1344.0f;
        break;
    case LSM303DLHCAccelerometer::ODR_5376HZ:
        f_odr = 5376.0f;
        break;
    }
    return f_odr;
}

void LSM303DLHCAccelerometer::set_full_scale(FullScale fs)
{
    _i2c_device.update_register(CTRL_REG4_A, fs, 0x30);

    // calculate m/s^2 / lsb
    switch (fs) {
    case LSM303DLHCAccelerometer::FULL_SCALE_2G:
        _sensitivity = 0.001f * GRAVITY_OF_EARTH;
        break;
    case LSM303DLHCAccelerometer::FULL_SCALE_4G:
        _sensitivity = 0.002f * GRAVITY_OF_EARTH;
        break;
    case LSM303DLHCAccelerometer::FULL_SCALE_8G:
        _sensitivity = 0.004f * GRAVITY_OF_EARTH;
        break;
    case LSM303DLHCAccelerometer::FULL_SCALE_16G:
        _sensitivity = 0.012f * GRAVITY_OF_EARTH;
        break;
    }
}

LSM303DLHCAccelerometer::FullScale LSM303DLHCAccelerometer::get_full_scale()
{
    uint8_t value = _i2c_device.read_register(CTRL_REG4_A, 0x30);
    FullScale fs;

    switch (value) {
    case 0x00:
        fs = FULL_SCALE_2G;
        break;
    case 0x10:
        fs = FULL_SCALE_4G;
        break;
    case 0x20:
        fs = FULL_SCALE_8G;
        break;
    case 0x30:
        fs = FULL_SCALE_16G;
        break;
    default:
        MBED_ERROR(MBED_ERROR_UNKNOWN, "Unreachable code");
    }
    return fs;
}

const float LSM303DLHCAccelerometer::GRAVITY_OF_EARTH = 9.80665f;

float LSM303DLHCAccelerometer::get_sensitivity()
{
    return _sensitivity;
}

void LSM303DLHCAccelerometer::set_high_pass_filter_mode(LSM303DLHCAccelerometer::HighPassFilterMode hpf)
{
    if (hpf == HPF_OFF) {
        _i2c_device.update_register(CTRL_REG2_A, 0x00, 0x08);
    } else {
        _i2c_device.update_register(CTRL_REG2_A, hpf | 0x08, 0x38);
    }
}

LSM303DLHCAccelerometer::HighPassFilterMode LSM303DLHCAccelerometer::get_high_pass_filter_mode()
{
    uint8_t val = _i2c_device.read_register(CTRL_REG2_A, 0x38);
    HighPassFilterMode hpf = HPF_OFF;
    if (val & 0x08) {
        switch (val & 0x30) {
        case 0x00:
            hpf = HPF_CF0;
            break;
        case 0x10:
            hpf = HPF_CF1;
            break;
        case 0x20:
            hpf = HPF_CF2;
            break;
        case 0x30:
            hpf = HPF_CF3;
            break;
        }
    }
    return hpf;
}

float LSM303DLHCAccelerometer::get_high_pass_filter_cut_off_frequency()
{
    uint8_t val = _i2c_device.read_register(CTRL_REG2_A, 0x30);
    float hp_c = val >> 4;
    float f_s = get_output_data_rate_hz();

    float f_cutt_off = -logf(1.0f - 3.0f / (25.0f * powf(2, hp_c))) * f_s / (2 * 3.14159265358979323846f);

    return f_cutt_off;
}

void LSM303DLHCAccelerometer::set_fifo_mode(LSM303DLHCAccelerometer::FIFOMode mode)
{
    if (mode) {
        _i2c_device.update_register(FIFO_CTRL_REG_A, 0x80, 0xC0); // configure FIFO stream mode
        _i2c_device.update_register(CTRL_REG5_A, 0x40, 0x40); // enable FIFO
    } else {
        _i2c_device.update_register(CTRL_REG5_A, 0x00, 0x40); // disabled FIFO
        _i2c_device.update_register(FIFO_CTRL_REG_A, 0x00, 0xC0); // configure FIFO bypass mode
    }
    // update drdy interrupt
    _process_interrupt_register(2);
}

LSM303DLHCAccelerometer::FIFOMode LSM303DLHCAccelerometer::get_fifo_mode()
{
    uint8_t fifo_mode = _i2c_device.read_register(CTRL_REG5_A, 0x40);
    if (fifo_mode) {
        return FIFO_ENABLE;
    } else {
        return FIFO_DISABLE;
    }
}

void LSM303DLHCAccelerometer::set_fifo_watermark(int watermark)
{
    if (watermark < 0 || watermark >= 32) {
        MBED_ERROR(MBED_ERROR_INVALID_ARGUMENT, "Invalid watermark value");
    }
    _i2c_device.update_register(FIFO_CTRL_REG_A, watermark, 0x1F);
}

int LSM303DLHCAccelerometer::get_fifo_watermark()
{
    return _i2c_device.read_register(FIFO_CTRL_REG_A, 0x1F);
}

void LSM303DLHCAccelerometer::clear_fifo()
{
    uint8_t fifo_mode = _i2c_device.read_register(FIFO_CTRL_REG_A, 0xC0);
    if (fifo_mode != 0) {
        // switch to bypass mode and back
        // (this action will clear FIFO)
        _i2c_device.update_register(FIFO_CTRL_REG_A, 0x00, 0xC0);
        _i2c_device.update_register(FIFO_CTRL_REG_A, fifo_mode, 0xC0);
    }
}

void LSM303DLHCAccelerometer::set_data_ready_interrupt_mode(LSM303DLHCAccelerometer::DatadaReadyInterruptMode drdy_mode)
{
    _process_interrupt_register(drdy_mode == DRDY_ENABLE ? 1 : 0);
}

LSM303DLHCAccelerometer::DatadaReadyInterruptMode LSM303DLHCAccelerometer::get_data_ready_interrupt_mode()
{
    return _process_interrupt_register(3);
}

void LSM303DLHCAccelerometer::set_high_resolution_output_mode(HighResolutionOutputMode hro)
{
    _i2c_device.update_register(CTRL_REG4_A, hro == HRO_ENABLED ? 0x08 : 0x00, 0x08);
}

LSM303DLHCAccelerometer::HighResolutionOutputMode LSM303DLHCAccelerometer::get_high_resolution_output_mode()
{
    return _i2c_device.read_register(CTRL_REG4_A, 0x08) ? HRO_ENABLED : HRO_DISABLED;
}

void LSM303DLHCAccelerometer::read_data(float data[3])
{
    int16_t data_16[3];
    read_data_16(data_16);
    for (int i = 0; i < 3; i++) {
        data[i] = data_16[i] * _sensitivity;
    }
}

void LSM303DLHCAccelerometer::read_data_16(int16_t data[3])
{
    uint8_t raw_data[6];
    _i2c_device.read_registers(OUT_X_L_A | 0x80, raw_data, 6);
    // data layout
    // - output resolution 12 bit
    // - assume that LSB is lower address, as it's default value
    //   note: the byte order is controlled by CTRL_REG4_A
    // - the value is left-justified, so we need to shift it to right
    data[0] = (int16_t)(raw_data[1] << 8 | raw_data[0]) >> 4; // X axis
    data[1] = (int16_t)(raw_data[3] << 8 | raw_data[2]) >> 4; // Y axis
    data[2] = (int16_t)(raw_data[5] << 8 | raw_data[4]) >> 4; // Z axis
}

void LSM303DLHCAccelerometer::_reboot_memory_content()
{
    _i2c_device.update_register(CTRL_REG5_A, 0x80, 0x80);
}

void LSM303DLHCAccelerometer::_dummy_read()
{
    uint8_t raw_data[6];
    _i2c_device.read_registers(OUT_X_L_A | 0x80, raw_data, 6);
}

void LSM303DLHCAccelerometer::_clear_data()
{
    uint8_t status = _i2c_device.read_register(STATUS_REG_A);
    if (status) {
        _dummy_read();
    }
}

LSM303DLHCAccelerometer::DatadaReadyInterruptMode LSM303DLHCAccelerometer::_process_interrupt_register(int mode)
{
    DatadaReadyInterruptMode res;
    FIFOMode fifo_mode;

    // CTRL_REG3_A bits:
    // 0b000000x0 - I1_OVERRUN - FIFO overrun
    // 0b00000x00 - I1_WTM - FIFO watermark
    // 0b0000x000 - I1_DRDY2 - purpose is unknown
    // 0b000x0000 - I1_DRDY1 - new data is generated
    switch (mode) {
    case 0:
        // disable interrupts
        _i2c_device.update_register(CTRL_REG3_A, 0x00, 0x1E);
        res = DRDY_DISABLE;
        break;
    case 1:
        // enable interrupt
        fifo_mode = get_fifo_mode();
        if (fifo_mode) {
            // watermark interrupt
            _i2c_device.update_register(CTRL_REG3_A, 0x04, 0x1E);
        } else {
            // DRDY interrupt
            _i2c_device.update_register(CTRL_REG3_A, 0x10, 0x1E);
        }
        _clear_data();
        res = DRDY_ENABLE;
        break;
    case 2:
        // update interrupt mode
        // note: it can be used if we switch off/of FIFO
        if (_process_interrupt_register(3) == DRDY_ENABLE) {
            res = _process_interrupt_register(1);
        } else {
            res = _process_interrupt_register(0);
        }
        break;
    case 3:
        // check current interrupt state
        if (_i2c_device.read_register(CTRL_REG3_A, 0x1E)) {
            res = DRDY_ENABLE;
        } else {
            res = DRDY_DISABLE;
        }
        break;
    }

    return res;
}
