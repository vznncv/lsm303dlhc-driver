/**
 * Example of the LSM303DLHC usage with STM32F3Discovery board.
 *
 * Base magnetometer usage.
 *
 * Pin map:
 *
 * - PC_4 - UART TX (stdout/stderr)
 * - PC_5 - UART RX (stdin)
 * - PB_7 - I2C SDA of the LSM303DLHC
 * - PB_6 - I2C SCL of the LSM303DLHC
 * - PE_2 - DRDY pin of the LSM303DLHC
 */
#include "lsm303dlhc_driver.h"
#include "math.h"
#include "mbed.h"

DigitalOut led(LED2);

int main()
{
    // create I2C interface separately. It allows to use it with different drivers.
    I2C mag_i2c(PB_7, PB_6);
    mag_i2c.frequency(400000); // LSM303DLHC can use I2C fast mode
    LSM303DLHCMagnetometer magnetometer(&mag_i2c);
    // perform basic configuration of the accelerometer (set default frequency, enable axes, etc.)
    int err_code = magnetometer.init();
    if (err_code) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, err_code), "magnetometer initialization error");
    }

    float mag_data[3];
    float abs_val, x, y, z;

    while (true) {
        // read accelerometer data in the m/s^2
        magnetometer.read_data(mag_data);
        x = mag_data[0];
        y = mag_data[1];
        z = mag_data[2];
        abs_val = sqrtf(x * x + y * y + z * z);

        printf("-----------------------------\n");
        printf("x:              %+.4f gauss\n", x);
        printf("y:              %+.4f gauss\n", y);
        printf("z:              %+.4f gauss\n", z);
        printf("absolute value: %+.4f gauss\n", abs_val);
        led = !led;
        wait(0.5);
    }
}
