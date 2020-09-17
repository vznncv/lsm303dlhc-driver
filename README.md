# LSM303DLHC interface library

The library contains an accelerometer and a magnetometer driver of the
[LSM303DLHC](https://www.st.com/en/mems-and-sensors/lsm303dlhc.html) chip for mbed-os.

Library allows:

- change output data range
- enable data ready interrupt (accelerometer only)
- set scale mode
- configure high pass filter (accelerometer only)
- read temperature value

The library is tested and and compatible with Mbed OS 6.3.

## Driver usage

Typical interface usage contains the following steps:

1. create `I2C` interface. This allows to use it with other drivers;
2. create `LSM303DLHCAccelerometer`/`LSM303DLHCMagnetometer` driver instances;
3. invoke `init` method. This method will perform basic device configuration, and set some default setting;
4. invoke driver method to configure LSM303DLHC for you purposes;
5. read data using `read_data` or `read_data_16` methods.

The simple program that uses accelerometer with [STM32F3Discovery](https://www.st.com/en/evaluation-tools/stm32f3discovery.html)
board is shown bellow:

```
/**
 * Example of the LSM303DLHC usage with STM32F3Discovery board.
 *
 * Base example of the LSM303DLHC usage.
 *
 * Pin map:
 *
 * - PC_4 - UART TX (stdout/stderr)
 * - PC_5 - UART RX (stdin)
 * - PB_7 - I2C SDA of the LSM303DLHC
 * - PB_6 - I2C SCL of the LSM303DLHC
 * - PE_4 - INT1 pin of the LSM303DLHC
 * - PE_2 - DRDY pin of the LSM303DLHC
 */
#include "lsm303dlhc_driver.h"
#include "math.h"
#include "mbed.h"

DigitalOut led(LED2);

int main()
{
    // create I2C interface separately. It allows to use it with different drivers.
    I2C lsm303dlhc_i2c(PB_7, PB_6);
    lsm303dlhc_i2c.frequency(400000); // LSM303DLHC can use I2C fast mode
    // instantiate magnetometer and accelerometer
    LSM303DLHCMagnetometer magnetometer(&lsm303dlhc_i2c);
    magnetometer.init();
    LSM303DLHCAccelerometer accelerometer(&lsm303dlhc_i2c);
    accelerometer.init();

    // sensor data
    float acc_data[3];
    float x_a, y_a, z_a, abs_a;
    float mag_data[3];
    float x_m, y_m, z_m, abs_m;

    while (true) {
        // read sensor data
        accelerometer.read_data(acc_data);
        x_a = acc_data[0];
        y_a = acc_data[1];
        z_a = acc_data[2];
        abs_a = sqrtf(x_a * x_a + y_a * y_a + z_a * z_a);
        magnetometer.read_data(mag_data);
        x_m = mag_data[0];
        y_m = mag_data[1];
        z_m = mag_data[2];
        abs_m = sqrtf(x_m * x_m + y_m * y_m + z_m * z_m);

        // print data to serial port
        printf("accelerometer: x = %+.4f m/s^2; y = %+.4f m/s^2; z = %+.4f m/s^2; abs = %+.4f m/s^2\n", x_a, y_a, z_a, abs_a);
        printf("magnetometer:  x = %+.4f gauss; y = %+.4f gauss; z = %+.4f gauss; abs = %+.4f gauss\n", x_m, y_m, z_m, abs_m);

        led = !led;
        ThisThread::sleep_for(1000ms);
    }
}
```

Other examples can be found in the folder `examples`.

## Run tests

The project contains some tests. To run them you should:

1. create a new project, with this library;
2. adjust `lsm303dlhc-driver.test_*` pins in the `mbed_json.app` for I2C and interrupts if you don't use a STM32F3Discovery board; 
3. connect board;
4. run `mbed test --greentea --test-by-name "lsm303dlhc-driver-tests-*"`.
