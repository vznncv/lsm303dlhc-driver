# LSM303DLHC accelerometer interface library

The library contains an accelerometer driver of the LSM303DLHC chip for mbed-os.
Library allows:

- change output data range
- enable data ready interrupt
- set scale mode
- configure high pass filter

## Accelerometer usage

Typical interface usage contains the following steps:

1. create `I2C` interface. This allows to use it with other drivers;
2. create `LSM303DLHCAccelerometer` driver instance;
3. invoke `LSM303DLHCAccelerometer::init` method. This method will perform 
   basic device configuration, and set some default setting;
4. configure LSM303DLHC for you purposes;
5. read data using `LSM303DLHCAccelerometer::read_data` or
   `LSM303DLHCAccelerometer::read_data_16` methods.

The simple program that uses accelerometer with STM32F3Discovery board is shown bellow:

```
/**
 * Base example of the LSM303DLHC usage with STM32F3Discovery board.
 *
 * Base accelerometer usage.
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

DigitalOut led(LED2);

int main()
{
    // Create I2C interface separately. It allows to use it with different drivers.
    I2C acc_i2c(PB_7, PB_6);
    acc_i2c.frequency(400000); // LSM303DLHC can use I2C fast mode

    LSM303DLHCAccelerometer accelerometer(&acc_i2c);
    // perform basic configuration of the accelerometer (set default frequency, enable axes, etc.)
    accelerometer.init();

    float acc_data[3];

    while (true) {
        // read accelerometer data in the m/s^2
        accelerometer.read_data(acc_data);
        printf("----------------\n");
        printf("x: %+.4f m/s^2\n", acc_data[0]);
        printf("y: %+.4f m/s^2\n", acc_data[1]);
        printf("z: %+.4f m/s^2\n", acc_data[2]);

        led = !led;
        wait(2.0);
    }
}
```

Other examples can be found in the folder `examples`.


