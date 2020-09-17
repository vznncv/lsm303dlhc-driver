/**
 * Example of the LSM303DLHC usage with STM32F3Discovery board.
 *
 * Example of the interrupt and FIFO usage.
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

struct accel_interrupt_processor_t {
    int count;
    LSM303DLHCAccelerometer *accel_ptr;
    DigitalOut *led_ptr;
    int block_size;

    void read_and_print()
    {
        led_ptr->write(1);

        float axes_data[3];
        for (int i = 0; i < block_size; i++) {
            accel_ptr->read_data(axes_data);
            printf("%4d. x = %+6.2f m/s^2; y = %+6.2f m/s^2; z = %+6.2f m/s^2\n", count, axes_data[0], axes_data[1], axes_data[2]);
            count++;
        }

        led_ptr->write(0);
    }
};

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

    printf("-- start accelerometer test --\n");
    InterruptIn int2(PE_4);
    DigitalOut led(LED2);
    EventQueue queue;
    int block_size = 10;
    accel_interrupt_processor_t accel_interrupt_processor = { .count = 0, .accel_ptr = &accelerometer, .led_ptr = &led, .block_size = block_size };
    Event<void()> read_and_print_event = queue.event(&accel_interrupt_processor, &accel_interrupt_processor_t::read_and_print);
    int2.rise(callback(&read_and_print_event, &Event<void()>::call));
    accelerometer.set_output_data_rate(LSM303DLHCAccelerometer::ODR_10HZ);
    accelerometer.set_fifo_mode(LSM303DLHCAccelerometer::FIFO_ENABLE);
    accelerometer.set_fifo_watermark(block_size);
    accelerometer.set_data_ready_interrupt_mode(LSM303DLHCAccelerometer::DRDY_ENABLE);
    queue.dispatch_forever();
}
