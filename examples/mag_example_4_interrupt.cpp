/**
 * Example of the LSM303DLHC usage with STM32F3Discovery board.
 *
 * Example of the interrupt usage.
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
#include "mbed.h"

class MagDataPrinter {
public:
    MagDataPrinter(LSM303DLHCMagnetometer* mag_ptr)
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
    LSM303DLHCMagnetometer* mag_ptr;
};

int main()
{
    // magnetometer initialization
    I2C mag_i2c(PB_7, PB_6);
    mag_i2c.frequency(400000);
    LSM303DLHCMagnetometer magnetometer(&mag_i2c);
    int err_code = magnetometer.init();
    if (err_code) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, err_code), "magnetometer initialization error");
    }

    InterruptIn drdy(PE_2);
    EventQueue queue;
    MagDataPrinter mag_data_printer(&magnetometer);
    magnetometer.set_output_data_rate(LSM303DLHCMagnetometer::ODR_3_0_HZ);
    drdy.rise(queue.event(&mag_data_printer, &MagDataPrinter::read_and_print));
    queue.dispatch_forever();
}
