#include "lsm303dhlc_driver_base.h"
#include "math.h"
#include "mbed_error.h"

LSM303DLHCAccelerometerBase::LSM303DLHCAccelerometerBase()
    : unit_per_lsb(0)
{
}

void LSM303DLHCAccelerometerBase::update_register(uint8_t reg, uint8_t val, uint8_t mask)
{
    uint8_t reg_val = read_register(reg);
    reg_val &= ~mask;
    val = val & mask;
    reg_val |= val;
    write_register(reg, reg_val);
}

uint8_t LSM303DLHCAccelerometerBase::read_register(uint8_t reg, uint8_t mask)
{
    return read_register(reg) & mask;
}

LSM303DLHCAccelerometerBase::~LSM303DLHCAccelerometerBase()
{
}

int LSM303DLHCAccelerometerBase::init()
{
    // check device id
    int device_id = read_register(WHO_AM_I_ADDR);
    if (device_id != DEVICE_ID) {
        MBED_WARNING(MBED_ERROR_INITIALIZATION_FAILED, "Invalid accelerometer id");
    }

    // set default modes
    set_data_ready_interrupt_mode(DRDY_DISABLE);
    set_full_scale(FULL_SCALE_2G);
    set_high_pass_filter_mode(HPF_OFF);
    set_high_resolution_output_mode(HRO_ENABLED);
    set_power_mode(NORMAL_POWER_MODE);
    set_output_data_rate(ODR_25HZ);

    return MBED_SUCCESS;
}

void LSM303DLHCAccelerometerBase::set_power_mode(PowerMode power_mode)
{
    // update power mode bit
    update_register(CTRL_REG1_A, (uint8_t)(power_mode << 3), 0x08);
}

LSM303DLHCAccelerometerBase::PowerMode LSM303DLHCAccelerometerBase::get_power_mode()
{
    uint8_t val = read_register(CTRL_REG1_A, 0x08);
    return val ? LOW_POWER_MODE : NORMAL_POWER_MODE;
}

void LSM303DLHCAccelerometerBase::set_output_data_rate(OutputDataRate odr)
{
    PowerMode power_mode = get_power_mode();

    if (odr == ODR_NONE) {
        update_register(CTRL_REG1_A, 0x00, 0x08);
    } else {
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
        // set ODR and enable axes
        update_register(CTRL_REG1_A, (odr & 0xF0) | 0x07, 0xF7);
    }
}

LSM303DLHCAccelerometerBase::OutputDataRate LSM303DLHCAccelerometerBase::get_output_data_rate()
{
    uint8_t val = read_register(CTRL_REG1_A, 0xF0);
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

float LSM303DLHCAccelerometerBase::get_output_data_rate_hz()
{
    float f_odr;

    switch (get_output_data_rate()) {
    case LSM303DLHCAccelerometerBase::ODR_NONE:
        f_odr = 0;
        break;
    case LSM303DLHCAccelerometerBase::ODR_1HZ:
        f_odr = 1.0f;
        break;
    case LSM303DLHCAccelerometerBase::ODR_10HZ:
        f_odr = 10.0f;
        break;
    case LSM303DLHCAccelerometerBase::ODR_25HZ:
        f_odr = 25.0f;
        break;
    case LSM303DLHCAccelerometerBase::ODR_50HZ:
        f_odr = 50.0f;
        break;
    case LSM303DLHCAccelerometerBase::ODR_100HZ:
        f_odr = 100.0f;
        break;
    case LSM303DLHCAccelerometerBase::ODR_200HZ:
        f_odr = 200.0f;
        break;
    case LSM303DLHCAccelerometerBase::ODR_400HZ:
        f_odr = 400.0f;
        break;
    case LSM303DLHCAccelerometerBase::ODR_1620HZ:
        f_odr = 1620.0f;
        break;
    case LSM303DLHCAccelerometerBase::ODR_1344HZ:
        f_odr = 1344.0f;
        break;
    case LSM303DLHCAccelerometerBase::ODR_5376HZ:
        f_odr = 5376.0f;
        break;
    }
    return f_odr;
}

void LSM303DLHCAccelerometerBase::set_full_scale(FullScale fs)
{
    update_register(CTRL_REG4_A, fs, 0x30);

    // calculate m/s^2 / lsb
    switch (fs) {
    case LSM303DLHCAccelerometerBase::FULL_SCALE_2G:
        unit_per_lsb = 0.001f * GRAVITY_OF_EARTH;
        break;
    case LSM303DLHCAccelerometerBase::FULL_SCALE_4G:
        unit_per_lsb = 0.002f * GRAVITY_OF_EARTH;
        break;
    case LSM303DLHCAccelerometerBase::FULL_SCALE_8G:
        unit_per_lsb = 0.004f * GRAVITY_OF_EARTH;
        break;
    case LSM303DLHCAccelerometerBase::FULL_SCALE_16G:
        unit_per_lsb = 0.012f * GRAVITY_OF_EARTH;
        break;
    }
}

LSM303DLHCAccelerometerBase::FullScale LSM303DLHCAccelerometerBase::get_full_scale()
{
    uint8_t value = read_register(CTRL_REG4_A, 0x30);
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

float LSM303DLHCAccelerometerBase::get_unit_per_lsb()
{
    return unit_per_lsb;
}

void LSM303DLHCAccelerometerBase::set_high_pass_filter_mode(LSM303DLHCAccelerometerBase::HighPassFilterMode hpf)
{
    if (hpf == HPF_OFF) {
        update_register(CTRL_REG2_A, 0x00, 0x08);
    } else {
        update_register(CTRL_REG2_A, hpf | 0x08, 0x38);
    }
}

LSM303DLHCAccelerometerBase::HighPassFilterMode LSM303DLHCAccelerometerBase::get_high_pass_filter_mode()
{
    uint8_t val = read_register(CTRL_REG2_A, 0x38);
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

float LSM303DLHCAccelerometerBase::get_high_pass_filter_cut_off_frequency()
{
    uint8_t val = read_register(CTRL_REG2_A, 0x30);
    float hp_c = val >> 4;
    float f_s = get_output_data_rate_hz();

    float f_cutt_off = -logf(1.0f - 3.0f / (25.0f * powf(2, hp_c))) * f_s / (2 * 3.14159265358979323846f);

    return f_cutt_off;
}

void LSM303DLHCAccelerometerBase::set_data_ready_interrupt_mode(LSM303DLHCAccelerometerBase::DatadaReadyInterruptMode drdy_mode)
{
    // Note: enable I1_DRDY1 interrupt
    // there is I2_DRDY2 interrupt bit, that can be set,
    // but I don't find what it means
    update_register(CTRL_REG3_A, drdy_mode == DRDY_ENABLE ? 0x10 : 0x00, 0x18);
}

LSM303DLHCAccelerometerBase::DatadaReadyInterruptMode LSM303DLHCAccelerometerBase::get_data_ready_interrupt_mode()
{
    uint8_t val = read_register(CTRL_REG3_A, 0x18);
    return val ? DRDY_ENABLE : DRDY_DISABLE;
}

void LSM303DLHCAccelerometerBase::set_high_resolution_output_mode(HighResolutionOutputMode hro)
{
    update_register(CTRL_REG4_A, hro == HRO_ENABLED ? 0x08 : 0x00, 0x08);
}

LSM303DLHCAccelerometerBase::HighResolutionOutputMode LSM303DLHCAccelerometerBase::get_high_resolution_output_mode()
{
    return read_register(CTRL_REG4_A, 0x08) ? HRO_ENABLED : HRO_DISABLED;
}

void LSM303DLHCAccelerometerBase::read_data(float data[3])
{
    int16_t data_16[3];
    read_data_16(data_16);
    for (int i = 0; i < 3; i++) {
        data[i] = data_16[i] * unit_per_lsb;
    }
}

void LSM303DLHCAccelerometerBase::read_data_16(int16_t data[3])
{
    uint8_t raw_data[6];
    read_registers(OUT_X_L_A, raw_data, 6);
    // data layout
    // - output resolution 12 bit
    // - assume that LSB is lower address, as it's default value
    //   note: the byte order is controlled by CTRL_REG4_A
    // - the value is left-justified, so we need to shift it to right
    data[0] = (int16_t)(raw_data[1] << 8 | raw_data[0]) >> 4; // X axis
    data[1] = (int16_t)(raw_data[3] << 8 | raw_data[2]) >> 4; // Y axis
    data[2] = (int16_t)(raw_data[5] << 8 | raw_data[4]) >> 4; // Z axis
}
