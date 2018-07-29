/**
 * Example of the LSM303DLHC usage with STM32F3Discovery board.
 *
 * Resolution and precision settings.
 *
 * Pin map:
 *
 * - PC_4 - UART TX (stdout/stderr)
 * - PC_5 - UART RX (stdin)
 * - PB_7 - I2C SDA of the LSM303DLHC
 * - PB_6 - I2C SCL of the LSM303DLHC
 * - PE_4 - INT1 pin of the LSM303DLHC
 */
#include "lsm303dlhc_driver.h"
#include "mbed.h"

void print_axis_val(const char* axis_name, int16_t value)
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

void read_and_print_accelerometer_data(LSM303DLHCAccelerometer* acc)
{
    int16_t acc_data[3];
    // read raw accelerometer data
    acc->read_data_16(acc_data);
    printf("-------------------------\n");
    print_axis_val("x", acc_data[0]);
    print_axis_val("y", acc_data[1]);
    print_axis_val("z", acc_data[2]);
}

DigitalOut led(LED2);

int main()
{
    // accelerometer initialization
    I2C acc_i2c(PB_7, PB_6);
    acc_i2c.frequency(400000);
    LSM303DLHCAccelerometer accelerometer(&acc_i2c);
    int err_code = accelerometer.init();
    if (err_code) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, err_code), "accelerometer initialization error");
    }

    const int n_repeat = 5;
    const float delay = 1.0;
    while (true) {
        accelerometer.set_full_scale(LSM303DLHCAccelerometer::FULL_SCALE_2G);
        accelerometer.set_high_resolution_output_mode(LSM303DLHCAccelerometer::HRO_ENABLED);
        printf("\nHigh resolution mode. Full scale - 2g\n");
        wait(delay);
        for (int i = 0; i < n_repeat; i++) {
            read_and_print_accelerometer_data(&accelerometer);
            led = !led;
            wait(delay);
        };

        accelerometer.set_full_scale(LSM303DLHCAccelerometer::FULL_SCALE_8G);
        accelerometer.set_high_resolution_output_mode(LSM303DLHCAccelerometer::HRO_ENABLED);
        printf("\nHigh resolution mode. Full scale - 8g\n");
        wait(delay);
        for (int i = 0; i < n_repeat; i++) {
            read_and_print_accelerometer_data(&accelerometer);
            led = !led;
            wait(delay);
        };

        accelerometer.set_full_scale(LSM303DLHCAccelerometer::FULL_SCALE_2G);
        accelerometer.set_high_resolution_output_mode(LSM303DLHCAccelerometer::HRO_DISABLED);
        printf("\nLow resolution mode. Full scale - 2g\n");
        wait(delay);
        for (int i = 0; i < n_repeat; i++) {
            read_and_print_accelerometer_data(&accelerometer);
            led = !led;
            wait(delay);
        };

        accelerometer.set_full_scale(LSM303DLHCAccelerometer::FULL_SCALE_8G);
        accelerometer.set_high_resolution_output_mode(LSM303DLHCAccelerometer::HRO_DISABLED);
        printf("\nLow resolution mode. Full scale - 8g\n");
        wait(delay);
        for (int i = 0; i < n_repeat; i++) {
            read_and_print_accelerometer_data(&accelerometer);
            led = !led;
            wait(delay);
        };
    }
}
