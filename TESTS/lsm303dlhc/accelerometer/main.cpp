#include "greentea-client/test_env.h"
#include "lsm303dlhc_driver.h"
#include "math.h"
#include "mbed.h"
#include "rtos.h"
#include "unity.h"
#include "utest.h"

using namespace utest::v1;
using namespace lsm303dlhc;

static LSM303DLHCAccelerometer *acc;

utest::v1::status_t test_setup_handler(const size_t number_of_cases)
{
    acc = new LSM303DLHCAccelerometer(MBED_CONF_LSM303DLHC_DRIVER_TEST_I2C_SDA, MBED_CONF_LSM303DLHC_DRIVER_TEST_I2C_SCL);
    return greentea_test_setup_handler(number_of_cases);
}

void test_teardown_handler(const size_t passed, const size_t failed, const failure_t failure)
{
    delete acc;
    return greentea_test_teardown_handler(passed, failed, failure);
}

utest::v1::status_t case_setup_handler(const Case *const source, const size_t index_of_case)
{
    // reset accelerometer
    acc->init();
    return greentea_case_setup_handler(source, index_of_case);
}

/**
 * Test accelerometer state after initialization.
 */
void test_init_state()
{
    int err;

    // check that init isn't failed
    err = acc->init();
    TEST_ASSERT_EQUAL(0, err);

    // check parameter
    TEST_ASSERT_EQUAL(LSM303DLHCAccelerometer::NORMAL_POWER_MODE, acc->get_power_mode());
    TEST_ASSERT_EQUAL(LSM303DLHCAccelerometer::ODR_25HZ, acc->get_output_data_rate());
    TEST_ASSERT_EQUAL(LSM303DLHCAccelerometer::HRO_ENABLED, acc->get_high_resolution_output_mode());
    TEST_ASSERT_EQUAL(LSM303DLHCAccelerometer::HPF_OFF, acc->get_high_pass_filter_mode());
    TEST_ASSERT_EQUAL(LSM303DLHCAccelerometer::FULL_SCALE_2G, acc->get_full_scale());
    TEST_ASSERT_EQUAL(LSM303DLHCAccelerometer::DRDY_DISABLE, acc->get_data_ready_interrupt_mode());
}

float abs_acc_val(float a_data[3])
{
    return sqrtf(a_data[0] * a_data[0] + a_data[1] * a_data[1] + a_data[2] * a_data[2]);
}

int16_t abs_acc_val(int16_t a_data[3])
{
    return sqrtf(a_data[0] * a_data[0] + a_data[1] * a_data[1] + a_data[2] * a_data[2]);
}

/**
 * Test accelerometer full scale modes.
 */
void test_full_scale()
{
    float a_vec[3];
    float a_abs;
    int16_t a_vec_int[3];
    int16_t a_abs_int;

    LSM303DLHCAccelerometer::FullScale fs_modes[4] = {
        LSM303DLHCAccelerometer::FULL_SCALE_2G, LSM303DLHCAccelerometer::FULL_SCALE_4G,
        LSM303DLHCAccelerometer::FULL_SCALE_8G, LSM303DLHCAccelerometer::FULL_SCALE_16G
    };
    int16_t expected_a_abs_ints[] = { 1024, 512, 256, 128 };
    int16_t expected_a_delta_ints[] = { 128, 64, 32, 48 };

    // check full scale modes
    for (int i = 0; i < 4; i++) {
        LSM303DLHCAccelerometer::FullScale fs_mode = fs_modes[i];
        int16_t expected_a_abs_int = expected_a_abs_ints[i];
        int16_t expected_a_delta_int = expected_a_delta_ints[i];

        acc->set_full_scale(fs_mode);
        wait_ms(200);

        acc->read_data(a_vec);
        a_abs = abs_acc_val(a_vec);
        TEST_ASSERT_FLOAT_WITHIN(1.0f, 9.8f, a_abs);
        acc->read_data_16(a_vec_int);
        a_abs_int = abs_acc_val(a_vec_int);
        TEST_ASSERT_INT_WITHIN(expected_a_delta_int, expected_a_abs_int, a_abs_int);
    }
}

struct simple_interrupt_counter_t {
    int count;
    float a_abs_sum;

    void process_interrupt()
    {
        count++;
        float a_vec[3];
        float a_abs;

        acc->read_data(a_vec);
        a_abs = abs_acc_val(a_vec);
        a_abs_sum += a_abs;
    }
};

/**
 * Test interrupt usage.
 */
void test_simple_iterrupt_usage()
{
    InterruptIn drdy_pin(MBED_CONF_LSM303DLHC_DRIVER_TEST_INT_1);
    simple_interrupt_counter_t interrupt_counter = { .count = 0, .a_abs_sum = 0 };
    Callback<void()> interrupt_cb = mbed_highprio_event_queue()->event(callback(&interrupt_counter, &simple_interrupt_counter_t::process_interrupt));
    drdy_pin.rise(interrupt_cb);

    // prepare accelerometer and run interrupts
    acc->set_output_data_rate(LSM303DLHCAccelerometer::ODR_25HZ);
    acc->set_data_ready_interrupt_mode(LSM303DLHCAccelerometer::DRDY_ENABLE);
    wait_ms(500);
    // disable interrupts
    acc->set_data_ready_interrupt_mode(LSM303DLHCAccelerometer::DRDY_DISABLE);
    drdy_pin.disable_irq();
    wait_ms(100);

    // check results
    TEST_ASSERT(interrupt_counter.count > 10);
    float a_abs = interrupt_counter.a_abs_sum / interrupt_counter.count;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 9.8f, a_abs);
}

// test cases description
#define AccCase(test_fun) Case(#test_fun, case_setup_handler, test_fun, greentea_case_teardown_handler, greentea_case_failure_continue_handler)
Case cases[] = {
    AccCase(test_init_state),
    AccCase(test_full_scale),
    AccCase(test_simple_iterrupt_usage)
};
Specification specification(test_setup_handler, cases, test_teardown_handler);

// Entry point into the tests
int main()
{
    // host handshake
    // note: should be invoked here or in the test_setup_handler
    GREENTEA_SETUP(40, "default_auto");
    // run tests
    return !Harness::run(specification);
}
