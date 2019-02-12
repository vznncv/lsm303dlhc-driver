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
void test_init_state()
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
    wait_ms(100);

    const int N_SAMPLES = 5;
    float samples[N_SAMPLES];
    float m_data[3];

    for (int i = 0; i < N_SAMPLES; i++) {
        mag->read_data(m_data);
        samples[i] = abs_mag_val(m_data);
        wait_ms(100);
    }

    // check that samples are different due noise
    for (int i = 0; i < N_SAMPLES; i++) {
        TEST_ASSERT_NOT_EQUAL(samples[i - 1], samples[i]);
    }
}

// test cases description
#define MagCase(test_fun) Case(#test_fun, case_setup_handler, test_fun, greentea_case_teardown_handler, greentea_case_failure_continue_handler)
Case cases[] = {
    MagCase(test_init_state),
    MagCase(test_temp_sensor),
    MagCase(test_magnetometer)
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
