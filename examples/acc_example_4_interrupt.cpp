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
 * - LSM303DLHC_INT1 - INT1 pin of the LSM303DLHC
 */
#define LSM303DLHC_I2C_SDA_PIN PB_7
#define LSM303DLHC_I2C_SCL_PIN PB_6
#define LSM303DLHC_INT1 PE_4

struct accel_interrupt_processor_t {
    int count;
    LSM303DLHCAccelerometer *accel_ptr;
    DigitalOut *led_ptr;

    accel_interrupt_processor_t(int count, LSM303DLHCAccelerometer *accel_ptr, DigitalOut *led_ptr)
        : count(count)
        , accel_ptr(accel_ptr)
        , led_ptr(led_ptr)
    {
    }

    void read_and_print()
    {
        led_ptr->write(1);

        float axes_data[3];
        accel_ptr->read_data(axes_data);
        printf("%4d. x = %+6.2f m/s^2; y = %+6.2f m/s^2; z = %+6.2f m/s^2\n", count, axes_data[0], axes_data[1], axes_data[2]);
        count++;

        led_ptr->write(0);
    }
};

int main()
{
    // accelerometer initialization
    I2C acc_i2c(LSM303DLHC_I2C_SDA_PIN, LSM303DLHC_I2C_SCL_PIN);
    acc_i2c.frequency(400000);
    LSM303DLHCAccelerometer accelerometer(&acc_i2c);
    int err_code = accelerometer.init();
    if (err_code) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, err_code), "accelerometer initialization error");
    }

    printf("-- start accelerometer test --\n");
    InterruptIn int2(LSM303DLHC_INT1);
    DigitalOut led(LED2);
    EventQueue queue;
    accel_interrupt_processor_t accel_interrupt_processor(0, &accelerometer, &led);
    int2.rise(queue.event(&accel_interrupt_processor, &accel_interrupt_processor_t::read_and_print));
    accelerometer.set_output_data_rate(LSM303DLHCAccelerometer::ODR_10HZ);
    accelerometer.set_data_ready_interrupt_mode(LSM303DLHCAccelerometer::DRDY_ENABLE);
    queue.dispatch_forever();
}
