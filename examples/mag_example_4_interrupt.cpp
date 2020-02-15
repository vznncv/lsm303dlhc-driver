/**
 * Example of the LSM303DLHC usage with STM32F3Discovery board.
 *
 * Example of the interrupt usage.
 */
#include "lsm303dlhc_driver.h"
#include "mbed.h"

/**
 * Pin map:
 *
 * - LSM303DLHC_I2C_SDA_PIN - I2C SDA of the LSM303DLHC
 * - LSM303DLHC_I2C_SCL_PIN - I2C SCL of the LSM303DLHC
 * - LSM303DLHC_DRDY - DRDY pin of the LSM303DLHC
 */
#define LSM303DLHC_I2C_SDA_PIN PB_7
#define LSM303DLHC_I2C_SCL_PIN PB_6
#define LSM303DLHC_DRDY PE_2

class MagDataPrinter {
public:
    MagDataPrinter(LSM303DLHCMagnetometer *mag_ptr)
        : count(0)
        , led(LED2)
        , mag_ptr(mag_ptr)
    {
    }

    void read_and_print()
    {
        float axes_data[3];
        mag_ptr->read_data(axes_data);
        printf("%4d. x = %+6.3f G; y = %+6.3f G; z = %+6.3f G\n", count, axes_data[0], axes_data[1], axes_data[2]);
        count++;
    }

private:
    int count;
    DigitalOut led;
    LSM303DLHCMagnetometer *mag_ptr;
};

int main()
{
    // magnetometer initialization
    I2C mag_i2c(LSM303DLHC_I2C_SDA_PIN, LSM303DLHC_I2C_SCL_PIN);
    mag_i2c.frequency(400000);
    LSM303DLHCMagnetometer magnetometer(&mag_i2c);
    int err_code = magnetometer.init();
    if (err_code) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, err_code), "magnetometer initialization error");
    }

    InterruptIn drdy(LSM303DLHC_DRDY);
    EventQueue queue;
    MagDataPrinter mag_data_printer(&magnetometer);
    magnetometer.set_output_data_rate(LSM303DLHCMagnetometer::ODR_3_0_HZ);
    drdy.rise(queue.event(&mag_data_printer, &MagDataPrinter::read_and_print));
    queue.dispatch_forever();
}
