#include "greentea-client/test_env.h"
#include "lsm303dlhc_driver.h"
#include "math.h"
#include "mbed.h"
#include "rtos.h"
#include "unity.h"
#include "utest.h"

using namespace utest::v1;
using namespace lsm303dlhc;

static LSM303DLHCMagnetometer *mag;

utest::v1::status_t test_setup_handler(const size_t number_of_cases)
{
    mag = new LSM303DLHCMagnetometer(MBED_CONF_LSM303DLHC_DRIVER_TEST_I2C_SDA, MBED_CONF_LSM303DLHC_DRIVER_TEST_I2C_SCL);
    return greentea_test_setup_handler(number_of_cases);
}

void test_teardown_handler(const size_t passed, const size_t failed, const failure_t failure)
{
    delete mag;
    return greentea_test_teardown_handler(passed, failed, failure);
}

utest::v1::status_t case_setup_handler(const Case *const source, const size_t index_of_case)
{
    // reset magnetometer
    mag->init();
    return greentea_case_setup_handler(source, index_of_case);
}

/**
 * Test magnetometer state after initialization.
 */
void test_init_state_enabled()
{
    int err;

    // check that init isn't failed
    err = mag->init();
    TEST_ASSERT_EQUAL(0, err);

    // check parameter
    TEST_ASSERT_EQUAL(LSM303DLHCMagnetometer::M_ENABLE, mag->get_magnetometer_mode());
    TEST_ASSERT_EQUAL(LSM303DLHCMagnetometer::ODR_15_HZ, mag->get_output_data_rate());
    TEST_ASSERT_EQUAL(LSM303DLHCMagnetometer::FULL_SCALE_1_3_G, mag->get_full_scale());
    TEST_ASSERT_EQUAL(LSM303DLHCMagnetometer::TS_ENABLE, mag->get_temperature_sensor_mode());
}

/**
 * Test magnetometer state after initialization.
 */
void test_init_state_disabled()
{
    int err;

    // check that init isn't failed
    err = mag->init(false);
    TEST_ASSERT_EQUAL(0, err);

    // check parameter
    TEST_ASSERT_EQUAL(LSM303DLHCMagnetometer::M_DISABLE, mag->get_magnetometer_mode());
    TEST_ASSERT_EQUAL(LSM303DLHCMagnetometer::ODR_15_HZ, mag->get_output_data_rate());
    TEST_ASSERT_EQUAL(LSM303DLHCMagnetometer::FULL_SCALE_1_3_G, mag->get_full_scale());
    TEST_ASSERT_EQUAL(LSM303DLHCMagnetometer::TS_ENABLE, mag->get_temperature_sensor_mode());
}

/**
 * Test temperature senors.
 */
void test_temp_sensor()
{
    int16_t temp;

    temp = mag->read_temperature_16();

    TEST_ASSERT_NOT_EQUAL(temp, 0);
}

float abs_mag_val(float m_data[3])
{
    return sqrtf(m_data[0] * m_data[0] + m_data[1] * m_data[1] + m_data[2] * m_data[2]);
}

/**
 * Common magnetometer test.
 */
void test_magnetometer()
{
    mag->set_output_data_rate(LSM303DLHCMagnetometer::ODR_15_HZ);
    ThisThread::sleep_for(100);

    const int N_SAMPLES = 5;
    float samples[N_SAMPLES];
    float m_data[3];

    for (int i = 0; i < N_SAMPLES; i++) {
        mag->read_data(m_data);
        samples[i] = abs_mag_val(m_data);
        ThisThread::sleep_for(100);
    }

    // check that samples are different due noise
    for (int i = 0; i < N_SAMPLES; i++) {
        TEST_ASSERT_NOT_EQUAL(samples[i - 1], samples[i]);
    }
}

struct interrupt_counter_t {
    int samples_count;
    int invokation_count;
    float m_abs_sum;

    const int samples_per_invokation;

    interrupt_counter_t(int samples_count, int invokation_count, float m_abs_sum, int samples_per_invokation)
        : samples_count(samples_count)
        , invokation_count(invokation_count)
        , m_abs_sum(m_abs_sum)
        , samples_per_invokation(samples_per_invokation)
    {
    }

    void process_interrupt()
    {
        invokation_count++;
        float m_vec[3];
        float m_abs;

        for (int i = 0; i < samples_per_invokation; i++) {
            samples_count++;

            mag->read_data(m_vec);
            m_abs = abs_mag_val(m_vec);
            m_abs_sum += m_abs;
        }
    }
};

/**
 * Test magnetometer interrupt usage.
 */
void test_magnetometer_interrupt()
{
    InterruptIn drdy_pin(MBED_CONF_LSM303DLHC_DRIVER_TEST_DRDY);
    interrupt_counter_t interrupt_counter(0, 0, 0.0f, 1);
    Callback<void()> interrupt_cb = mbed_highprio_event_queue()->event(callback(&interrupt_counter, &interrupt_counter_t::process_interrupt));

    // prepare accelerometer and run interrupts
    mag->set_output_data_rate(LSM303DLHCMagnetometer::ODR_30_HZ);
    drdy_pin.rise(interrupt_cb);

    // wait processing
    ThisThread::sleep_for(500);

    // disable interrupts
    drdy_pin.disable_irq();
    ThisThread::sleep_for(500);

    // check results
    TEST_ASSERT(interrupt_counter.samples_count > 10);
    TEST_ASSERT(interrupt_counter.samples_count < 20);
    float m_abs = interrupt_counter.m_abs_sum / interrupt_counter.samples_count;

    TEST_ASSERT(m_abs > 0.01);
}

// test cases description
#define MagCase(test_fun) Case(#test_fun, case_setup_handler, test_fun, greentea_case_teardown_handler, greentea_case_failure_continue_handler)
Case cases[] = {
    MagCase(test_init_state_enabled),
    MagCase(test_init_state_disabled),
    MagCase(test_temp_sensor),
    MagCase(test_magnetometer),
    MagCase(test_magnetometer_interrupt)
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
