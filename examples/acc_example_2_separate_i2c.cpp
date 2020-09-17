/**
 * Base example of the LSM303DLHC usage with STM32F3Discovery board.
 *
 * Base accelerometer usage.
 */
#include "lsm303dlhc_driver.h"
#include "mbed.h"

/**
 * Pin map:
 *
 * - LSM303DLHC_I2C_SDA_PIN - I2C SDA of the LSM303DLHC
 * - LSM303DLHC_I2C_SCL_PIN - I2C SCL of the LSM303DLHC
 */
#define LSM303DLHC_I2C_SDA_PIN PB_7
#define LSM303DLHC_I2C_SCL_PIN PB_6

DigitalOut led(LED2);

int main()
{
    // create I2C interface separately. It allows to use it with different drivers.
    I2C acc_i2c(LSM303DLHC_I2C_SDA_PIN, LSM303DLHC_I2C_SCL_PIN);
    acc_i2c.frequency(400000); // LSM303DLHC can use I2C fast mode
    LSM303DLHCAccelerometer accelerometer(&acc_i2c);
    // perform basic configuration of the accelerometer (set default frequency, enable axes, etc.)
    int err_code = accelerometer.init();
    if (err_code) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, err_code), "accelerometer initialization error");
    }

    float acc_data[3];

    while (true) {
        // read accelerometer data in the m/s^2
        accelerometer.read_data(acc_data);
        printf("----------------\n");
        printf("x: %+.4f m/s^2\n", acc_data[0]);
        printf("y: %+.4f m/s^2\n", acc_data[1]);
        printf("z: %+.4f m/s^2\n", acc_data[2]);

        led = !led;
        ThisThread::sleep_for(2000ms);
    }
}
