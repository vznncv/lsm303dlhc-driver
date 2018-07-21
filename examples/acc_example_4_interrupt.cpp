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
 * - PE_4 - INT1 pin of the LSM303DLHC
 */
#include "lsm303dhlc_driver.h"
#include "mbed.h"

class AccelDataPrinter {
public:
    AccelDataPrinter(LSM303DLHCAccelerometer* accel_ptr)
        : count(0)
        , led(LED2)
        , accel_ptr(accel_ptr)
    {
    }

    void read_and_print()
    {
        float axes_data[3];
        accel_ptr->read_data(axes_data);
        printf("%4d. x = %+6.2f m/s^2; y = %+6.2f m/s^2; z = %+6.2f m/s^2\n", count, axes_data[0], axes_data[1], axes_data[2]);
        count++;
    }

private:
    int count;
    DigitalOut led;
    LSM303DLHCAccelerometer* accel_ptr;
};

int main()
{
    // accelerometer initialization
    I2C acc_i2c(PB_7, PB_6);
    acc_i2c.frequency(400000);
    LSM303DLHCAccelerometer accelerometer(&acc_i2c);
    accelerometer.init();

    InterruptIn int2(PE_4);
    EventQueue queue;
    AccelDataPrinter accel_data_printer(&accelerometer);
    accelerometer.set_output_data_rate(LSM303DLHCAccelerometer::ODR_10HZ);
    int2.rise(queue.event(&accel_data_printer, &AccelDataPrinter::read_and_print));
    accelerometer.set_data_ready_interrupt_mode(LSM303DLHCAccelerometer::DRDY_ENABLE);
    queue.dispatch_forever();
}
