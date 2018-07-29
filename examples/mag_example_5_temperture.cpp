/**
 * Example of the LSM303DLHC usage with STM32F3Discovery board.
 *
 * Example of the temperature sensor usage.
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
    I2C acc_i2c(PB_7, PB_6);
    acc_i2c.frequency(400000); // LSM303DLHC can use I2C fast mode
    LSM303DLHCMagnetometer magnetometer(&acc_i2c);
    // perform basic configuration of the accelerometer (set default frequency, enable axes, etc.)
    int err_code = magnetometer.init();
    if (err_code) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, err_code), "magnetometer initialization error");
    }

    float temperture;
    int16_t temperture_16;

    while (true) {
        // read current temperture
        temperture = magnetometer.read_temperature();
        temperture_16 = magnetometer.read_temperature_16();

        printf("-----------------------------\n");
        printf("temperature:       %+.4f C\n", temperture);
        printf("temperature (raw): 0x%04X \n", temperture_16);
        led = !led;
        wait(1.0);
    }
}
