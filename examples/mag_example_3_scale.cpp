/**
 * Example of the LSM303DLHC usage with STM32F3Discovery board.
 *
 * Precision settings example.
 */
#include "lsm303dlhc_driver.h"
#include "mbed.h"
#include <chrono>

/**
 * Pin map:
 *
 * - LSM303DLHC_I2C_SDA_PIN - I2C SDA of the LSM303DLHC
 * - LSM303DLHC_I2C_SCL_PIN - I2C SCL of the LSM303DLHC
 */
#define LSM303DLHC_I2C_SDA_PIN PB_7
#define LSM303DLHC_I2C_SCL_PIN PB_6

void print_axis_val(const char *axis_name, int16_t value)
{
    // convert value binary representation to show precision/resolution settings
    int sign = 1;
    if (value < 0) {
        sign = -1;
        value = -value;
    }

    char buff[32];
    int pos = 0;
    for (int i = 0; i < 16; i++) {
        buff[pos] = value & 0x8000 ? '1' : '0';
        value <<= 1;
        pos++;
        if (i % 4 == 3) {
            buff[pos] = '_';
            pos++;
        }
    }
    buff[pos - 1] = '\0';

    printf("%s: %c0b%s\n", axis_name, sign >= 0 ? '+' : '-', buff);
}

void read_and_print_magnetometer_data(LSM303DLHCMagnetometer *mag)
{
    int16_t mag_data[3];
    // read raw magnetometer data
    mag->read_data_16(mag_data);
    printf("-------------------------\n");
    print_axis_val("x", mag_data[0]);
    print_axis_val("y", mag_data[1]);
    print_axis_val("z", mag_data[2]);
}

DigitalOut led(LED2);

int main()
{
    // magnetometer initialization
    I2C mag_i2c(LSM303DLHC_I2C_SDA_PIN, LSM303DLHC_I2C_SCL_PIN);
    mag_i2c.frequency(400000); // LSM303DLHC can use I2C fast mode
    LSM303DLHCMagnetometer magnetometer(&mag_i2c);
    int err_code = magnetometer.init();
    if (err_code) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, err_code), "magnetometer initialization error");
    }

    const int n_repeat = 5;
    const chrono::milliseconds delay_ms = 1000ms;
    while (true) {
        magnetometer.set_full_scale(LSM303DLHCMagnetometer::FULL_SCALE_1_3_G);
        printf("\nFull scale - 1.4 Gauss\n");
        ThisThread::sleep_for(delay_ms);
        for (int i = 0; i < n_repeat; i++) {
            read_and_print_magnetometer_data(&magnetometer);
            led = !led;
            ThisThread::sleep_for(delay_ms);
        };

        magnetometer.set_full_scale(LSM303DLHCMagnetometer::FULL_SCALE_8_1_G);
        printf("\nFull scale - 8.1 Gauss\n");
        ThisThread::sleep_for(delay_ms);
        for (int i = 0; i < n_repeat; i++) {
            read_and_print_magnetometer_data(&magnetometer);
            led = !led;
            ThisThread::sleep_for(delay_ms);
        };
    }
}
