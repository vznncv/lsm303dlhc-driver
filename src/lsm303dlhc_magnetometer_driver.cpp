#include "lsm303dlhc_magnetometer_driver.h"

using namespace lsm303dlhc;

LSM303DLHCMagnetometer::LSM303DLHCMagnetometer(I2C *i2c_ptr)
    : i2c_device(I2C_ADDRESS, i2c_ptr)
    , xy_mag_sensitivity(0)
    , z_mag_sensitivity(0)
{
}

LSM303DLHCMagnetometer::LSM303DLHCMagnetometer(PinName sda, PinName scl, int frequency)
    : i2c_device(I2C_ADDRESS, sda, scl, frequency)
    , xy_mag_sensitivity(0)
    , z_mag_sensitivity(0)
{
}

LSM303DLHCMagnetometer::~LSM303DLHCMagnetometer()
{
}

int LSM303DLHCMagnetometer::init(bool start)
{
    // check IRx_REG_M register
    if (i2c_device.read_register(IRA_REG_M) != IRA_REG_M_VAL) {
        return MBED_ERROR_INITIALIZATION_FAILED;
    } else if (i2c_device.read_register(IRB_REG_M) != IRB_REG_M_VAL) {
        return MBED_ERROR_INITIALIZATION_FAILED;
    } else if (i2c_device.read_register(IRC_REG_M) != IRC_REG_M_VAL) {
        return MBED_ERROR_INITIALIZATION_FAILED;
    }

    // set default values
    set_temperature_sensor_mode(TS_ENABLE);
    set_output_data_rate(ODR_15_HZ);
    set_full_scale(FULL_SCALE_1_3_G);
    set_magnetometer_mode(start ? M_ENABLE : M_DISABLE);

    return MBED_SUCCESS;
}

uint8_t LSM303DLHCMagnetometer::read_register(uint8_t reg)
{
    return i2c_device.read_register(reg);
}

void LSM303DLHCMagnetometer::write_register(uint8_t reg, uint8_t val)
{
    i2c_device.write_register(reg, val);
}

void LSM303DLHCMagnetometer::set_temperature_sensor_mode(TemperatureSensorMode tsm)
{
    i2c_device.update_register(CRA_REG_M, tsm, 0x80);
}

LSM303DLHCMagnetometer::TemperatureSensorMode LSM303DLHCMagnetometer::get_temperature_sensor_mode()
{
    uint8_t val = i2c_device.read_register(CRA_REG_M, 0x80);
    return val ? TS_ENABLE : TS_DISABLE;
}

float LSM303DLHCMagnetometer::read_temperature()
{
    int16_t t_raw = read_temperature_16();
    return t_raw * temperature_sensitivity + temperature_offset;
}

int16_t LSM303DLHCMagnetometer::read_temperature_16()
{
    uint8_t data[2];
    i2c_device.read_registers(TEMP_OUT_H_M, data, 2);
    return (int16_t)(((int16_t)data[0] << 8) + data[1]) >> 4;
}

float LSM303DLHCMagnetometer::get_temperature_sensor_sensitivity()
{
    return temperature_sensitivity;
}

float LSM303DLHCMagnetometer::get_temperature_sensor_zero_offset()
{
    return temperature_offset;
}

void LSM303DLHCMagnetometer::set_output_data_rate(OutputDataRate odr)
{
    i2c_device.update_register(CRA_REG_M, (uint8_t)(odr << 2), 0x3C);
}

LSM303DLHCMagnetometer::OutputDataRate LSM303DLHCMagnetometer::get_output_data_rate()
{
    uint8_t val = i2c_device.read_register(CRA_REG_M, 0x3C) >> 2;
    OutputDataRate odr;

    switch (val) {
    case 0x00:
        odr = ODR_0_75_HZ;
        break;
    case 0x01:
        odr = ODR_1_5_HZ;
        break;
    case 0x02:
        odr = ODR_3_0_HZ;
        break;
    case 0x03:
        odr = ODR_7_5_HZ;
        break;
    case 0x04:
        odr = ODR_15_HZ;
        break;
    case 0x05:
        odr = ODR_30_HZ;
        break;
    case 0x06:
        odr = ODR_75_HZ;
        break;
    case 0x07:
        odr = ODR_220_HZ;
        break;
    default:
        MBED_ERROR(MBED_ERROR_UNKNOWN, "Unreachable code");
    }
    return odr;
}

float LSM303DLHCMagnetometer::get_output_data_rate_hz()
{
    OutputDataRate odr = get_output_data_rate();
    float res;

    switch (odr) {
    case lsm303dlhc::LSM303DLHCMagnetometer::ODR_0_75_HZ:
        res = 0.75f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::ODR_1_5_HZ:
        res = 1.5f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::ODR_3_0_HZ:
        res = 3.0f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::ODR_7_5_HZ:
        res = 7.5f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::ODR_15_HZ:
        res = 15.0f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::ODR_30_HZ:
        res = 30.0f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::ODR_75_HZ:
        res = 75.0f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::ODR_220_HZ:
        res = 220.0f;
        break;
    }
    return res;
}

void LSM303DLHCMagnetometer::set_magnetometer_mode(MagnetometerMode mm)
{
    i2c_device.update_register(MR_REG_M, mm ? 0x00 : 0x03, 0x03);
}

LSM303DLHCMagnetometer::MagnetometerMode LSM303DLHCMagnetometer::get_magnetometer_mode()
{
    uint8_t val = i2c_device.read_register(MR_REG_M, 0x03);
    return val & 0x02 ? M_DISABLE : M_ENABLE;
}

void LSM303DLHCMagnetometer::set_full_scale(FullScale fs)
{
    i2c_device.update_register(CRB_REG_M, fs, 0xE0);
    switch (fs) {
    case lsm303dlhc::LSM303DLHCMagnetometer::FULL_SCALE_1_3_G:
        xy_mag_sensitivity = 1.0f / 1100.0f;
        z_mag_sensitivity = 1.0f / 980.0f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::FULL_SCALE_1_9_G:
        xy_mag_sensitivity = 1.0f / 885.0f;
        z_mag_sensitivity = 1.0f / 760.0f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::FULL_SCALE_2_5_G:
        xy_mag_sensitivity = 1.0f / 670.0f;
        z_mag_sensitivity = 1.0f / 600.0f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::FULL_SCALE_4_0_G:
        xy_mag_sensitivity = 1.0f / 450.0f;
        z_mag_sensitivity = 1.0f / 400.0f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::FULL_SCALE_4_7_GA:
        xy_mag_sensitivity = 1.0f / 400.0f;
        z_mag_sensitivity = 1.0f / 355.0f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::FULL_SCALE_5_6_G:
        xy_mag_sensitivity = 1.0f / 330.0f;
        z_mag_sensitivity = 1.0f / 295.0f;
        break;
    case lsm303dlhc::LSM303DLHCMagnetometer::FULL_SCALE_8_1_G:
        xy_mag_sensitivity = 1.0f / 230.0f;
        z_mag_sensitivity = 1.0f / 205.0f;
        break;
    }
}

LSM303DLHCMagnetometer::FullScale LSM303DLHCMagnetometer::get_full_scale()
{
    uint8_t val = i2c_device.read_register(CRB_REG_M, 0xE0);
    FullScale fs;
    switch (val) {
    case 0x20:
        fs = FULL_SCALE_1_3_G;
        break;
    case 0x40:
        fs = FULL_SCALE_1_9_G;
        break;
    case 0x60:
        fs = FULL_SCALE_2_5_G;
        break;
    case 0x80:
        fs = FULL_SCALE_4_0_G;
        break;
    case 0xA0:
        fs = FULL_SCALE_4_7_GA;
        break;
    case 0xC0:
        fs = FULL_SCALE_5_6_G;
        break;
    case 0xE0:
        fs = FULL_SCALE_8_1_G;
        break;
    default:
        MBED_ERROR(MBED_ERROR_UNKNOWN, "Unreachable code");
    }
    return fs;
}

float LSM303DLHCMagnetometer::get_sensitivity(int axis_no)
{
    if (axis_no == 0) {
        return xy_mag_sensitivity;
    } else if (axis_no == 1) {
        return xy_mag_sensitivity;
    } else if (axis_no == 2) {
        return z_mag_sensitivity;
    } else {
        MBED_ERROR(MBED_ERROR_INVALID_ARGUMENT, "Invalid axis number");
    }
}

void LSM303DLHCMagnetometer::read_data(float data[])
{
    int16_t data_16[3];
    read_data_16(data_16);
    data[0] = data_16[0] * xy_mag_sensitivity;
    data[1] = data_16[1] * xy_mag_sensitivity;
    data[2] = data_16[2] * z_mag_sensitivity;
}

void LSM303DLHCMagnetometer::read_data_16(int16_t data[])
{
    uint8_t raw_data[6];
    i2c_device.read_registers(OUT_X_H_M, raw_data, 6);
    data[0] = (int16_t)((raw_data[0] << 8) + raw_data[1]);
    data[1] = (int16_t)((raw_data[4] << 8) + raw_data[5]);
    data[2] = (int16_t)((raw_data[2] << 8) + raw_data[3]);
}

const float LSM303DLHCMagnetometer::temperature_sensitivity = 1.0f / 16.0f;
const float LSM303DLHCMagnetometer::temperature_offset = 21.0f;
