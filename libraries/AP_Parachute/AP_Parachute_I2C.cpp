#include "AP_Parachute_I2C.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/sparse-endian.h>

#define AP_PARACHUTE_I2C_ADC_ADDR                           ((uint8_t)0x48)
#define AP_PARACHUTE_I2C_ADC_CONVERSION_REG                 ((uint8_t)0x00)
#define AP_PARACHUTE_I2C_ADC_CONFIGURATION_REG              ((uint8_t)0x01)
#define ADC_OS                                              ((uint16_t)(0x0001 << 15))  // OS       - 1 : Start a single conversion (when in a power-down state)
#define ADC_PGA                                             ((uint16_t)(0x0000 << 9))   // PGA[2:0] - 000 : FSR = ±6.144 V
#define ADC_MODE                                            ((uint16_t)(0x0001 << 8))   // MODE     - 1 : Single-shot conversion mode or power-down state (default)
#define ADC_DATA_RATE                                       ((uint16_t)(0x0007 << 5))   // DR[2:0]  - 111 : DR = 3300 SPS
#define ADC_RESERVED                                        ((uint16_t)(0x0003 << 0))   // Reserved - Always write 03h
#define AP_PARACHUTE_I2C_ADC_CONFIGURATION_VALUE            ((uint16_t)(0x0000 | ADC_OS | ADC_PGA | ADC_MODE | ADC_DATA_RATE | ADC_RESERVED))

#define AP_PARACHUTE_I2C_IO_EXPANDER_ADDR                   ((uint8_t)0x38)
#define AP_PARACHUTE_I2C_IO_EXPANDER_INPUT_REG              ((uint8_t)0x00)   
#define AP_PARACHUTE_I2C_IO_EXPANDER_OUTPUT_REG             ((uint8_t)0x01)   
#define AP_PARACHUTE_I2C_IO_EXPANDER_POLARITY_REG           ((uint8_t)0x02)   
#define AP_PARACHUTE_I2C_IO_EXPANDER_CONFIGURATION_REG      ((uint8_t)0x03)
#define PYRO_TEST_OUT                                       ((uint8_t)0x01)  // 0000 0001
#define PYRO_ARM_OUT                                        ((uint8_t)0x02)  // 0000 0010
#define PYRO_TRIG_OUT                                       ((uint8_t)0x04)  // 0000 0100
#define MOTOR_CUTOFF_OUT                                    ((uint8_t)0x08)  // 0000 1000
#define POWER_FAILURE_IN                                    ((uint8_t)0x10)  // 0001 0000
#define BUZZER_ENABLE_OUT                                   ((uint8_t)0x20)  // 0010 0000
#define SELF_TEST_OUT                                       ((uint8_t)0x40)  // 0100 0000
#define SELF_TEST_IN                                        ((uint8_t)0x80)  // 1000 0000
#define AP_PARACHUTE_I2C_IO_EXPANDER_OUTPUT_INITIAL_VALUE   ((uint8_t)0x00)  // initial output values. Bit values in this register have no effect on pins defined as inputs.
#define AP_PARACHUTE_I2C_IO_EXPANDER_CONFIGURATION_VALUE    ((uint8_t)(0x00 | POWER_FAILURE_IN | SELF_TEST_IN))  // 0 for OUT pin, 1 for IN pin

#define AP_PARACHUTE_I2C_SUPERCAP_ADDR                      ((uint8_t)0x09)
#define AP_PARACHUTE_I2C_SUPERCAP_CAP_ESR_PER_REG           ((uint8_t)0x04)     // each LSB represents 10 seconds. tests will not repeat if this register is zero.
#define AP_PARACHUTE_I2C_SUPERCAP_CTL_REG                   ((uint8_t)0x17)
#define CTL_STRT_CAPESR                                     ((uint16_t)0x0001)  // 0000 0000 0000 0001
#define CTL_STOP_CAPESR                                     ((uint16_t)0x0004)  // 0000 0000 0000 0100
#define AP_PARACHUTE_I2C_SUPERCAP_NUM_CAPS_REG              ((uint8_t)0x1A)
#define AP_PARACHUTE_I2C_SUPERCAP_MON_STATUS_REG            ((uint8_t)0x1C)
#define MON_CAPESR_ACTIVE                                   ((uint16_t)0x0001)  // 0000 0000 0000 0001
#define MON_CAPESR_PENDING                                  ((uint16_t)0x0004)  // 0000 0000 0000 0100
#define MON_CAP_DONE                                        ((uint16_t)0x0008)  // 0000 0000 0000 1000
#define MON_ESR_DONE                                        ((uint16_t)0x0010)  // 0000 0000 0001 0000
#define MON_CAP_FAILED                                      ((uint16_t)0x0020)  // 0000 0000 0010 0000
#define MON_ESR_FAILED                                      ((uint16_t)0x0040)  // 0000 0000 0100 0000
#define AP_PARACHUTE_I2C_SUPERCAP_CHRG_STATUS_REG           ((uint8_t)0x1B)
#define CHRG_STEPDOWN                                       ((uint16_t)0x0001)  // 0000 0000 0000 0001
#define CHRG_STEPUP                                         ((uint16_t)0x0002)  // 0000 0000 0000 0010
#define CHRG_CV                                             ((uint16_t)0x0004)  // 0000 0000 0000 0100
#define CHRG_UVLO                                           ((uint16_t)0x0008)  // 0000 0000 0000 1000
#define CHRG_INPUT_ILIM                                     ((uint16_t)0x0010)  // 0000 0000 0001 0000
#define CHRG_CAPPG                                          ((uint16_t)0x0020)  // 0000 0000 0010 0000
#define CHRG_SHNT                                           ((uint16_t)0x0040)  // 0000 0000 0100 0000
#define CHRG_BAL                                            ((uint16_t)0x0080)  // 0000 0000 1000 0000
#define CHRG_DIS                                            ((uint16_t)0x0100)  // 0000 0001 0000 0000
#define CHRG_CI                                             ((uint16_t)0x0200)  // 0000 0010 0000 0000
#define CHRG_PFO                                            ((uint16_t)0x0800)  // 0000 1000 0000 0000
#define AP_PARACHUTE_I2C_SUPERCAP_ALARM_REG                 ((uint8_t)0x1D)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_CAP_REG              ((uint8_t)0x1E)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_ESR_REG              ((uint8_t)0x1F)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_VCAP1_REG            ((uint8_t)0x20)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_VCAP2_REG            ((uint8_t)0x21)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_VCAP3_REG            ((uint8_t)0x22)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_VCAP4_REG            ((uint8_t)0x23)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_GPI_REG              ((uint8_t)0x24)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_VIN_REG              ((uint8_t)0x25)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_VCAP_REG             ((uint8_t)0x26)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_VOUT_REG             ((uint8_t)0x27)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_IIN_REG              ((uint8_t)0x28)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_ICHG_REG             ((uint8_t)0x29)
#define AP_PARACHUTE_I2C_SUPERCAP_MEAS_DTEMP_REG            ((uint8_t)0x2A)

#define AP_PARACHUTE_I2C_SUPERCAP_VCAPS_GPI_V_FACTOR_UV     183.5f                              // 183.5uV per LSB
#define AP_PARACHUTE_I2C_SUPERCAP_VIN_VOUT_FACTOR_MV        2.21f                               // 2.21mV per LSB
#define AP_PARACHUTE_I2C_SUPERCAP_CAP_STACK_FACTOR_MV       1.476f                              // 1.476mV per LSB
#define AP_PARACHUTE_I2C_SUPERCAP_IIN_ICHG_FACTOR_UV        1.983f                              // 1.983uV/RSNSI per LSB
#define AP_PARACHUTE_I2C_SUPERCAP_DTEMP_FACTOR_DEGC(value)  (((value) * (0.028f)) - (251.4f))   // 0.028degC per LSB - 251.4degC

#define AP_PARACHUTE_I2C_EXTENDER_ADDR                      ((uint8_t)0x3E)
#define AP_PARACHUTE_I2C_EXTENDER_EVENT_REG                 ((uint8_t)0x02)
#define EXTENDER_LINK_GOOD                                  ((uint8_t)0x01)     // 0000 0001
#define EXTENDER_LINK_LOST                                  ((uint8_t)0x02)     // 0000 0010
#define EXTENDER_FAULT                                      ((uint8_t)0x04)     // 0000 0100
#define AP_PARACHUTE_I2C_EXTENDER_FAULT_REG                 ((uint8_t)0x04)

#define ADC_MAX_WAIT_TIME_MS                                500
#define ADC_LSB_SIZE                                        (0.003f)            // corresponding LSB size for FSR = ±6.144 is 3mV
#define SUPERCAP_MEAS_CAP_FACTOR                            (336.0f * 1.0e-6f)  // 336μF * RT/RTST per LSB
#define SUPERCAP_ESR_FACTOR                                 (64.0f)             // RSNSC/64 per LSB
#define INFLIGHT_TESTS_RATE_MS                              (1000 / 5)          // 5Hz
#define PREFLIGHT_TESTS_RATE_MS                             (1000 / 1)          // 1Hz
#define SUPERCAP_CHARGE_STATUS_ERROR_MS                     3000                // after this amount of time of consecutive error, we fail the test
#define SUPERCAP_RESTART_TEST_MS                            20000               // after this amount of time we will restart the cap/esr tests

#define LOG_MAX_TEXT_LENGTH                                 50                  // max length of log message 
#define LOG_TEXT_LAST_SEND_ARRAY_MAX                        20                  // save crc of 20 last log messages
#define LOG_SEND_TEXT_MIN_TIME_DIFF_MS                      5000                // will not log same message before this time has passed 

#define MAX_SUPERCAP_REGS_FROZEN_TIME_US                    (5 * 1000 * 1000)           // time before declaring the supercaps registers as frozen
#define SUPERCAP_REGS_FROZEN_LOG_FREQ_US                    (45 * 1000 * 1000)          // min time between supercap registers frozen logs

static struct {
    uint32_t text_crc32;
    uint32_t last_sent_ms;
} log_text_last_sent[LOG_TEXT_LAST_SEND_ARRAY_MAX] = { 0 };

/*
   Detects if an I2C Parachute is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_Parachute_I2C *AP_Parachute_I2C::detect(AP_Parachute::_i2c_params_t& params)
{
    AP_Parachute_I2C *sensor = nullptr;
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();

    FOREACH_I2C(bus) {
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> adc_dev = hal.i2c_mgr->get_device(bus, AP_PARACHUTE_I2C_ADC_ADDR, params.bus_clock);
        if (!adc_dev) {
            continue;
        }
        adc_dev->set_split_transfers(true);

        AP_HAL::OwnPtr<AP_HAL::I2CDevice> io_expander_dev = hal.i2c_mgr->get_device(bus, AP_PARACHUTE_I2C_IO_EXPANDER_ADDR, params.bus_clock);
        if (!io_expander_dev) {
            continue;
        }
        io_expander_dev->set_split_transfers(true);

        AP_HAL::OwnPtr<AP_HAL::I2CDevice> supercap_dev = hal.i2c_mgr->get_device(bus, AP_PARACHUTE_I2C_SUPERCAP_ADDR, params.bus_clock);
        if (!supercap_dev) {
            continue;
        }
        supercap_dev->set_split_transfers(true);

        AP_HAL::OwnPtr<AP_HAL::I2CDevice> extender_dev = hal.i2c_mgr->get_device(bus, AP_PARACHUTE_I2C_EXTENDER_ADDR, params.bus_clock);
        if (!extender_dev) {
            continue;
        }
        extender_dev->set_split_transfers(false);

        sensor = new AP_Parachute_I2C(std::move(adc_dev), std::move(io_expander_dev), std::move(supercap_dev), std::move(extender_dev), params);

        if ((!sensor) || (!sensor->init())) {
            delete sensor;
            sensor = nullptr;
            continue;
        }
        sensor->write_to_log(true, MAV_SEVERITY_INFO, "AP_Parachute_I2C initialized for bus %lu", bus);
        break;
    }

    return sensor;
}

void AP_Parachute_I2C::write_to_log(MAV_SEVERITY severity, const char *fmt, va_list arg_list, bool force_write/*=false*/)
{
    WITH_SEMAPHORE(_log_write_semaphore);
    // making sure that we're not flooding the log
    uint32_t now_ms = AP_HAL::millis();
    char text[LOG_MAX_TEXT_LENGTH + 1];
    AP_HAL::get_HAL().util->vsnprintf(text, sizeof(text), fmt, arg_list);
    uint32_t text_crc = crc_crc32(0, (const uint8_t *)text, strlen(text));
    int write_index = -1;
    for (int i = 0; i < LOG_TEXT_LAST_SEND_ARRAY_MAX; ++i) {
        bool is_fresh = (now_ms - log_text_last_sent[i].last_sent_ms) <= LOG_SEND_TEXT_MIN_TIME_DIFF_MS;

        if (log_text_last_sent[i].text_crc32 != text_crc) {
            if ((write_index < 0) && (!is_fresh)) {
                write_index = i;
            }
            continue;
        }

        if (is_fresh && (!force_write)) {
            return;
        }

        write_index = i;
        break;
    }
    if (write_index >= 0) {
        log_text_last_sent[write_index].last_sent_ms = now_ms;
        log_text_last_sent[write_index].text_crc32 = text_crc;
    }

    gcs().send_text(severity, "%s", text);
}

void AP_Parachute_I2C::write_to_log(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    write_to_log(severity, fmt, arg_list);
    va_end(arg_list);
}

void AP_Parachute_I2C::write_to_log(bool force_write, MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    write_to_log(severity, fmt, arg_list, force_write);
    va_end(arg_list);
}

// void AP_Parachute_I2C::debug_i2c_channels(int8_t channels)
// {
//     io_expander_write_value(channels);
// }

bool AP_Parachute_I2C::write_reg(AP_HAL::OwnPtr<AP_HAL::I2CDevice>& dev, const uint8_t reg, const uint8_t* value, const uint8_t value_size)
{
    WITH_SEMAPHORE(dev->get_semaphore());
    uint8_t output_cmd[value_size + 1] = { reg };
    memcpy(output_cmd + 1, value, value_size);
    if (!dev->transfer(output_cmd, sizeof(output_cmd), nullptr, 0)) {
        return false;
    }
    return true;
}

bool AP_Parachute_I2C::read_reg(AP_HAL::OwnPtr<AP_HAL::I2CDevice>& dev, const uint8_t reg, const uint8_t value_size, uint8_t* value)
{
    WITH_SEMAPHORE(dev->get_semaphore());
    const uint8_t output_cmd[] = { reg };
    uint8_t input_value[value_size] = { 0 };
    if (!dev->transfer(output_cmd, sizeof(output_cmd), input_value, value_size)) {
        return false;
    }
    memcpy(value, input_value, value_size);
    return true;
}

bool AP_Parachute_I2C::io_expander_write_value(const uint8_t value)
{
    if (!write_reg(_io_expander_dev, AP_PARACHUTE_I2C_IO_EXPANDER_OUTPUT_REG, &value, sizeof(value))) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to write value 0x%X to io_expander", value);
        return false;
    }
    _io_expander_current_value = value;
    return true;
}

bool AP_Parachute_I2C::io_expander_read_value(uint8_t& value)
{
    if (!read_reg(_io_expander_dev, AP_PARACHUTE_I2C_IO_EXPANDER_INPUT_REG, sizeof(value), &value)) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to read from io_expander");
        return false;
    }
    return true;
}

bool AP_Parachute_I2C::adc_write_config(const int16_t value)
{
    be16_t value_be16 = htobe16(value);
    if (!write_reg(_adc_dev, AP_PARACHUTE_I2C_ADC_CONFIGURATION_REG, (uint8_t*)(&value_be16), sizeof(value_be16))) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to write value 0x%X to adc", value);
        return false;
    }
    return true;
}

bool AP_Parachute_I2C::adc_read_config(int16_t& value)
{
    if (!read_reg(_adc_dev, AP_PARACHUTE_I2C_ADC_CONFIGURATION_REG, sizeof(value), (uint8_t*)(&value))) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to read from adc config reg");
        return false;
    }
    return true;
}

bool AP_Parachute_I2C::adc_read_value(int16_t& value)
{
    be16_t input_value = 0;
    if (!read_reg(_adc_dev, AP_PARACHUTE_I2C_ADC_CONVERSION_REG, sizeof(input_value), (uint8_t*)(&input_value))) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to read from adc conversion reg");
        return false;
    }
    value = (be16toh(input_value) >> 4);  // 12 bits of data in binary two's-complement format that is left-justified within the 16-bit data word.
    return true;
}

bool AP_Parachute_I2C::adc_read_input(adc_input_t in, int16_t& value)
{
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    if (!adc_write_config(AP_PARACHUTE_I2C_ADC_CONFIGURATION_VALUE | (((uint16_t)in) << 12))) {
        return false;
    }

    uint32_t start_time_ms = AP_HAL::millis();
    int16_t input_value = 0;
    while (AP_HAL::millis() - start_time_ms < ADC_MAX_WAIT_TIME_MS) {
        hal.scheduler->delay(1);
        if (!adc_read_config(input_value)) {
            return false;
        }
        if (!(input_value & ADC_OS)) {
            continue;  // read isn't ready yet
        }
        if (!adc_read_value(input_value)) {
            return false;
        }
        value = input_value;
        return true;
    }
    return false;
}

bool AP_Parachute_I2C::supercap_write_reg(const uint8_t reg, const int16_t value)
{
    if (!write_reg(_supercap_dev, reg, (uint8_t*)(&value), sizeof(value))) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to write to supercap reg 0x%X", reg);
        return false;
    }
    return true;
}

bool AP_Parachute_I2C::supercap_read_reg(const uint8_t reg, int16_t& value)
{
    if (!read_reg(_supercap_dev, reg, sizeof(value), (uint8_t*)(&value))) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to read reg 0x%X from supercap", reg);
        return false;
    }
    return true;
}

bool AP_Parachute_I2C::init_io_expander()
{
    WITH_SEMAPHORE(_io_expander_dev->get_semaphore());
    const uint8_t output_cmd[] = { AP_PARACHUTE_I2C_IO_EXPANDER_OUTPUT_REG, AP_PARACHUTE_I2C_IO_EXPANDER_OUTPUT_INITIAL_VALUE };
    if (!_io_expander_dev->transfer(output_cmd, sizeof(output_cmd), nullptr, 0)) {
        return false;
    }
    _io_expander_current_value = AP_PARACHUTE_I2C_IO_EXPANDER_OUTPUT_INITIAL_VALUE;

    const uint8_t configuration_cmd[] = { AP_PARACHUTE_I2C_IO_EXPANDER_CONFIGURATION_REG, AP_PARACHUTE_I2C_IO_EXPANDER_CONFIGURATION_VALUE };
    if (!_io_expander_dev->transfer(configuration_cmd, sizeof(configuration_cmd), nullptr, 0)) {
        return false;
    }

    return true;
}

bool AP_Parachute_I2C::init()
{
    if (!init_io_expander()) {
        return false;
    }

    if (!extender_tests()) {
        return false;
    }

    // validating number of capacitors
    uint8_t capacitors_number = get_capacitors_number();
    if (capacitors_number != _params.capacitors.get()) {
        write_to_log(MAV_SEVERITY_WARNING, "number of capacitors: %d. should be: %d", capacitors_number, _params.capacitors.get());
        return false;
    }

    _cap_esr_test_started = false;

    _inflight_tests_results = false;

    AP_HAL::get_HAL().scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Parachute_I2C::inflight_tests_wrapper, void));

    return true;
}

void AP_Parachute_I2C::play_tone(uint32_t duration_ms)
{
    if (_play_tone_duration_ms != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "tried to play the buzzer for %lums while already playing a %lums buzzer", (unsigned long)duration_ms, (unsigned long)_play_tone_duration_ms);
        return;
    }

    _play_tone_start_ms = AP_HAL::millis();
    _play_tone_duration_ms = duration_ms;
}

bool AP_Parachute_I2C::buzzer(bool on_off)
{
    uint8_t value_to_write;
    if (on_off) {
        value_to_write = _io_expander_current_value | BUZZER_ENABLE_OUT;
    } else {
        value_to_write = _io_expander_current_value & (~BUZZER_ENABLE_OUT);
    }
    if (!io_expander_write_value(value_to_write)) {
        return false;
    }
    return true;    
}

bool AP_Parachute_I2C::release_prepare()
{
    if (_params.simulate) {
        write_to_log(true, MAV_SEVERITY_ERROR, "i2c parachute initiated SIMULATION %u", AP_HAL::millis());
        return true;
    }

    write_to_log(true, MAV_SEVERITY_ERROR, "I2C PARACHUTE INITIATING %u", AP_HAL::millis());
    if (!io_expander_write_value(_io_expander_current_value | BUZZER_ENABLE_OUT | MOTOR_CUTOFF_OUT)) {
        return false;
    }
    write_to_log(true, MAV_SEVERITY_ERROR, "I2C PARACHUTE INITIATED %u", AP_HAL::millis());
    return true;
}

bool AP_Parachute_I2C::release()
{
    bool res = true;
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();

    if (_params.simulate) {
        write_to_log(true, MAV_SEVERITY_ERROR, "i2c parachute released SIMULATION %u", AP_HAL::millis());
        return true;
    }

    write_to_log(true, MAV_SEVERITY_ERROR, "I2C PARACHUTE RELEASING %u", AP_HAL::millis());

    for (int i = 0; i < 10; ++i) {
        if (!io_expander_write_value(_io_expander_current_value | PYRO_ARM_OUT | PYRO_TRIG_OUT | BUZZER_ENABLE_OUT | MOTOR_CUTOFF_OUT)) {
            res = false;
        }
        hal.scheduler->delay(10);

        // sample pyro voltages and log
        int16_t value = 0;
        if (!adc_read_input(adc_input_e::AIN0, value)) {
            write_to_log(true, MAV_SEVERITY_WARNING, "failed to read AIN0");
        }
        float ain0 = value * ADC_LSB_SIZE;
        write_to_log(true, MAV_SEVERITY_ERROR, "ain0 is : %.3fV", ain0);
        if (!adc_read_input(adc_input_e::AIN1, value)) {
            write_to_log(true, MAV_SEVERITY_WARNING, "failed to read AIN1");
        }
        float ain1 = value * ADC_LSB_SIZE;
        write_to_log(true, MAV_SEVERITY_ERROR, "ain1 is : %.3fV", ain1);

        hal.scheduler->delay(10);
        if (!io_expander_write_value(_io_expander_current_value & (~PYRO_ARM_OUT) & (~PYRO_TRIG_OUT))) {
            res = false;
        }
        hal.scheduler->delay(20);
    }
    write_to_log(true, MAV_SEVERITY_ERROR, "I2C PARACHUTE RELEASED %u", AP_HAL::millis());
    return res;
}

bool AP_Parachute_I2C::loopback_tests()
{
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    
    // set loopback to high and read back high
    uint8_t read_value = 0;
    if (!io_expander_write_value(_io_expander_current_value | SELF_TEST_OUT)) {
        return false;
    }
    hal.scheduler->delay(1);
    if (!io_expander_read_value(read_value)) {
        return false;
    }
    if ((read_value & SELF_TEST_IN) == 0) {
        write_to_log(MAV_SEVERITY_WARNING, "loopback high failed");
        return false;
    }

    // set loopback to low and read low
    read_value = 0xFF;
    if (!io_expander_write_value(_io_expander_current_value & (~SELF_TEST_OUT))) {
        return false;
    }
    hal.scheduler->delay(1);
    if (!io_expander_read_value(read_value)) {
        return false;
    }
    if ((read_value & SELF_TEST_IN) != 0) {
        write_to_log(MAV_SEVERITY_WARNING, "loopback low failed");
        return false;
    }

    return true;
}

uint8_t AP_Parachute_I2C::get_capacitors_number()
{
    int16_t value;
    if (!supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_NUM_CAPS_REG, value)) {
        return 0;        
    }
    return value + 1;

}

// The capacitance and capacitor ESR measurements must be initiated by setting the ctl_strt_capesr bit in the
// ctl_reg register. This bit will automatically clear once the measurement begins.
bool AP_Parachute_I2C::start_cap_esr_test()
{
    int16_t value;
    if (!supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_CTL_REG, value)) {
        return false;        
    }
    if (value & CTL_STRT_CAPESR) {
        _cap_esr_test_started = true;
        return true;  // cap_esr test already started
    }

    if (!supercap_write_reg(AP_PARACHUTE_I2C_SUPERCAP_CAP_ESR_PER_REG, 1)) {  // run cap_esr tests every 10 seconds
        write_to_log(true, MAV_SEVERITY_WARNING, "failed to schedule capacitance and ESR measurement period");
        return false;
    }
    if (!supercap_write_reg(AP_PARACHUTE_I2C_SUPERCAP_CTL_REG, value | CTL_STRT_CAPESR)) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to start cap_esr test");
        return false;
    }
    write_to_log(true, MAV_SEVERITY_INFO, "cap_esr test initiated");
    _cap_esr_test_started = true;
    return true;
}

// stop currently running CAP_ESR test (it might still be running from last flight)
bool AP_Parachute_I2C::stop_cap_esr_test()
{
    int16_t value;
    if (!supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_CTL_REG, value)) {
        return false;        
    }

    if (!supercap_write_reg(AP_PARACHUTE_I2C_SUPERCAP_CTL_REG, (value | CTL_STOP_CAPESR) & (~CTL_STRT_CAPESR))) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to stop cap_esr test");
        return false;
    }

    if (_cap_esr_test_started) {
        write_to_log(true, MAV_SEVERITY_INFO, "cap_esr test stopped");
        _cap_esr_test_started = false;
    }

    return true;
}

// runs test asynchronious and returns false only if test failed.
bool AP_Parachute_I2C::supercap_capacity_tests()
{
    static uint32_t last_test_finished = AP_HAL::millis();
    static bool last_test_result = true;
    static bool test_reported = true;
    int16_t value;

    if (!supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MON_STATUS_REG, value)) {
        return false;        
    }
    if ((value & MON_CAPESR_ACTIVE) || (value & MON_CAPESR_PENDING)) {
        if (test_reported) {
            write_to_log(true, MAV_SEVERITY_INFO, "cap_esr test started");
            test_reported = false;
            last_test_finished = 0;
        }
        return true;  // test is in progress
    }

    if ((last_test_finished != 0) && ((AP_HAL::millis() - last_test_finished) > SUPERCAP_RESTART_TEST_MS)) {
        write_to_log(true, MAV_SEVERITY_INFO, "cap_esr test restart");
        stop_cap_esr_test();  // stopping the test here and it will be restarted in the next iteration of inflight_tests
        last_test_finished = AP_HAL::millis();  // reset the counter so we can restart again if restart didn't work
        return true;  // it is not necessarily a problem that the test hasn't started
    }

    if (test_reported) {
        return last_test_result;
    }
    if ((value & MON_CAP_FAILED) || (value & MON_ESR_FAILED)) {
        write_to_log(true, MAV_SEVERITY_ERROR, "capacitor tests failed");
        test_reported = true;
        last_test_result = false;
        if (last_test_finished == 0) {
            last_test_finished = AP_HAL::millis();
        }
        return false;
    }

    if ((value & MON_CAP_DONE) && (value & MON_ESR_DONE)) {
        write_to_log(true, MAV_SEVERITY_INFO, "capacitor tests succeded");
        if (supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MEAS_CAP_REG, value)) {
            if (!is_zero(_params.rtst)){
                write_to_log(true, MAV_SEVERITY_DEBUG, "meas_cap: %.3f", value * (SUPERCAP_MEAS_CAP_FACTOR * _params.rt / _params.rtst));
            }
            else {
                write_to_log(MAV_SEVERITY_WARNING, "RTST param is zero. can't log meas_cap value properly.");
            }
        }

        if (supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MEAS_ESR_REG, value)) {
            write_to_log(true, MAV_SEVERITY_DEBUG, "meas_esr: %.3f", value * (_params.rsnsc / SUPERCAP_ESR_FACTOR));
        }
        test_reported = true;
        if (last_test_finished == 0) {
            last_test_finished = AP_HAL::millis();
        }
    }

    // waiting for next test
    last_test_result = true;
    return true;
}

bool AP_Parachute_I2C::is_supercap_status_valid()
{
    static uint32_t last_valid = 0;

    int16_t value;
    if (!supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_ALARM_REG, value)) {
        return false;        
    }
    if (value != 0) {
        write_to_log(MAV_SEVERITY_ERROR, "supercap alarm reg value: 0x%X", value);
        return false;
    }

    bool res = true;
    if (!supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_CHRG_STATUS_REG, value)) {
        return false;        
    }

    if (!(value & CHRG_STEPDOWN)) {
        write_to_log(MAV_SEVERITY_WARNING, "supercap is not in charging mode");
        res = false;
    }

    if (value & CHRG_STEPUP) {
        _backup_mode = true;
        write_to_log(MAV_SEVERITY_WARNING, "supercap is in backup mode");
        res = false;
    } else {
        _backup_mode = false;
    }

    if (value & CHRG_CV) {
        // write_to_log(MAV_SEVERITY_DEBUG, "supercap is in constant voltage mode");
    }

    if (value & CHRG_UVLO) {
        write_to_log(MAV_SEVERITY_WARNING, "supercap is in undervoltage lockout");
        res = false;
    }

    if (value & CHRG_INPUT_ILIM) {
        write_to_log(MAV_SEVERITY_WARNING, "supercap is in input current limit");
        res = false;
    }

    if (!(value & CHRG_CAPPG)) {
        write_to_log(MAV_SEVERITY_WARNING, "supercap is not above power good threshold");
        res = false;
    }

    if (value & CHRG_SHNT) {
        write_to_log(MAV_SEVERITY_INFO, "supercap is shunting");
    }

    if (value & CHRG_BAL) {
        // write_to_log(MAV_SEVERITY_INFO, "supercap is balancing");
    }

    if (value & CHRG_DIS) {
        // write_to_log(MAV_SEVERITY_DEBUG, "supercap is temporarily disabled for capacitance measurment");
    }

    if (value & CHRG_CI) {
        // write_to_log(MAV_SEVERITY_WARNING, "supercap is in constant current mode");
    }

    if (value & CHRG_PFO) {
        write_to_log(MAV_SEVERITY_WARNING, "supercap is below PFI threshold");
        res = false;
    }

    if (!res) {
        write_to_log(MAV_SEVERITY_DEBUG, "supercap charge status reg value: 0x%X", value);
    }

    uint32_t now = AP_HAL::millis();
    if (res) {
        last_valid = now;
    } else if ((last_valid > 0) && ((now - last_valid) < SUPERCAP_CHARGE_STATUS_ERROR_MS)) {
        res = true;  // we fail the test only after SUPERCAP_CHARGE_STATUS_ERROR_MS of consecutive errors
    }

    return res;
}

bool AP_Parachute_I2C::preflight_tests(char *failure_msg, const uint8_t failure_msg_len)
{

    if (!get_inflight_tests_results()) {
        strncpy(failure_msg, "i2c_parachute failed inflight_tests", failure_msg_len);
        _healthy = false;
        return _healthy;
    }

    // if (_state.frozen_since_us > 0) {
    //     _healthy = false;
    //     return _healthy;
    // }

    _healthy = true;
    return _healthy;
}

bool AP_Parachute_I2C::pyro_tests()
{
    static bool values_reported = false;
    bool res = true;
    int16_t value = 0;

    if (!io_expander_write_value(_io_expander_current_value & (~PYRO_TEST_OUT))) {
        goto bad;
    }

    if (!adc_read_input(adc_input_e::AIN2, value)) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to read AIN2");
        goto bad;
    }
    _bus_voltage = value * ADC_LSB_SIZE * 2.0f;
    if (_bus_voltage < _params.min_bus_voltage) {
        write_to_log(MAV_SEVERITY_ERROR, "bus is undervoltage: %.3fV", + _bus_voltage);
        goto bad;
    }

    if (!io_expander_write_value(_io_expander_current_value | PYRO_TEST_OUT)) {
        goto bad;
    }

    if (!adc_read_input(adc_input_e::AIN0, value)) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to read AIN0");
        goto bad;
    }
    _pyro_current = value * ADC_LSB_SIZE;
    if ((_pyro_current < (_params.ain_test_expected_voltage - _params.ain_test_expected_tolerance)) ||
        (_pyro_current > (_params.ain_test_expected_voltage + _params.ain_test_expected_tolerance))) {
        write_to_log(MAV_SEVERITY_ERROR, "pyro_current is not in range: %.3fV", _pyro_current);
        goto bad;
    }

    if (!adc_read_input(adc_input_e::AIN1, value)) {
        write_to_log(MAV_SEVERITY_WARNING, "failed to read AIN1");
        goto bad;
    }
    _pyro_short_current = value * ADC_LSB_SIZE;
    if ((_pyro_short_current < (_params.ain_test_expected_voltage - _params.ain_test_expected_tolerance)) ||
        (_pyro_short_current > (_params.ain_test_expected_voltage + _params.ain_test_expected_tolerance))) {
        write_to_log(MAV_SEVERITY_ERROR, "pyro short current is not in range: %.3fV", _pyro_short_current);
        goto bad;
    }

    if ((!AP::arming().is_armed()) && (!values_reported)) {
        write_to_log(true, MAV_SEVERITY_DEBUG, "ain0: %.3fV ain1: %.3fV ain2: %.3fV", _pyro_current, _pyro_short_current, _bus_voltage);
        values_reported = true;
    }

    goto cleanup;
bad:
    res = false;

cleanup:
    if (!io_expander_write_value(_io_expander_current_value & (~PYRO_TEST_OUT))) {
        res = false;
    }

    return res;
}

bool AP_Parachute_I2C::extender_tests()
{
    bool res = true;
    uint8_t event_value = 0;
    uint8_t fault_value = 0;

    if (!read_reg(_extender_dev, AP_PARACHUTE_I2C_EXTENDER_EVENT_REG, sizeof(event_value), &event_value)) {
        write_to_log(MAV_SEVERITY_ERROR, "can't read i2c extender event reg");
        res = false;
        goto cleanup;
    }
    if (event_value & EXTENDER_LINK_LOST) {
        write_to_log(MAV_SEVERITY_ERROR, "extender link lost");
        res = false;
        goto cleanup;
    }
    if (event_value & EXTENDER_FAULT) {
        if (!read_reg(_extender_dev, AP_PARACHUTE_I2C_EXTENDER_FAULT_REG, sizeof(fault_value), &fault_value)) {
            res = false;
            write_to_log(MAV_SEVERITY_ERROR, "can't read i2c extender fault reg");
            goto cleanup;
        }
        write_to_log(MAV_SEVERITY_DEBUG, "i2c extender fault reg: 0x%X", fault_value);
        goto cleanup;
    }

cleanup:
    if ((event_value & EXTENDER_LINK_LOST) || (event_value & EXTENDER_FAULT)) {
        event_value = 0;
        if (!write_reg(_extender_dev, AP_PARACHUTE_I2C_EXTENDER_EVENT_REG, &event_value, sizeof(event_value))) {
            write_to_log(MAV_SEVERITY_ERROR, "can't clear extender event reg");
        }
    }

    return res;
}

void AP_Parachute_I2C::inflight_tests_wrapper()
{
    const uint32_t now = AP_HAL::millis();
    static uint32_t last_run_ms = 0;
    const uint32_t tests_rate_ms = AP::arming().is_armed() ? INFLIGHT_TESTS_RATE_MS : PREFLIGHT_TESTS_RATE_MS;

    if (_play_tone_duration_ms > 0) {
        if (now - _play_tone_start_ms >= _play_tone_duration_ms) {
            buzzer(false);
            _play_tone_duration_ms = 0;
        } else {
            buzzer(true);
        }
    }

    if ((last_run_ms > 0) && ((now - last_run_ms) < tests_rate_ms)) {
        return;
    } 
    last_run_ms = now;
    
    _inflight_tests_results = inflight_tests();
}

// will be run once every PREFLIGHT_TESTS_RATE_MS before arming and every INFLIGHT_TESTS_RATE_MS after arming the FTS
bool AP_Parachute_I2C::inflight_tests()
{
    if (!loopback_tests()) {
        write_to_log(MAV_SEVERITY_ERROR, "i2c_parachute failed loopback_tests");
        _healthy = false;
        return _healthy;
    }

    update();
    write_log();

    if (!pyro_tests()) {
        write_to_log(MAV_SEVERITY_ERROR, "failed pyro tests");
        _healthy = false;
        return _healthy;
    }

    if (!extender_tests()) {
        write_to_log(MAV_SEVERITY_ERROR, "failed extender tests");
        _healthy = false;
        return _healthy;
    }

    if (!is_supercap_status_valid()) {
        if (!_params.ignore_cap_tests) {
            if (AP::arming().is_armed()) {
                write_to_log(MAV_SEVERITY_ERROR, "failed supercap status");
            }
            _healthy = false;
        }
        stop_cap_esr_test();
        return _healthy;
    }

    if(!_cap_esr_test_started) {
        start_cap_esr_test();
    }
    if (!supercap_capacity_tests()) {
        if (!_params.ignore_cap_tests) {
            if (AP::arming().is_armed()) {
                write_to_log(MAV_SEVERITY_ERROR, "failed supercap capacity tests");
            }
            _healthy = false;
            return _healthy;
        }
    }

    // if (_state.frozen_since_us > 0) {
    //     _healthy = false;
    //     return _healthy;
    // }

    _healthy = true;
    return _healthy;
}

void AP_Parachute_I2C::update()
{
    int16_t value;

    struct parachute_i2c_state state = { 0 };
    bool is_frozen = true;

    for (uint8_t i = 0; i < MAX_CAPACITORS; ++i) {
        if (supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MEAS_VCAP1_REG + i, value)) {
            state.vcap_single[i] = value * AP_PARACHUTE_I2C_SUPERCAP_VCAPS_GPI_V_FACTOR_UV;
            if (!is_equal(_state.vcap_single[i], state.vcap_single[i])) {
                is_frozen = false;
            }
        }
    }
    if (supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MEAS_GPI_REG, value)) {
        state.gpi = value * AP_PARACHUTE_I2C_SUPERCAP_VCAPS_GPI_V_FACTOR_UV;
        if (!is_equal(_state.gpi, state.gpi)) {
            is_frozen = false;
        }
    }
    if (supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MEAS_VIN_REG, value)) {
        state.vin = value * AP_PARACHUTE_I2C_SUPERCAP_VIN_VOUT_FACTOR_MV;
        if (!is_equal(_state.vin, state.vin)) {
            is_frozen = false;
        }
    }
    if (supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MEAS_VCAP_REG, value)) {
        state.vcap_total = value * AP_PARACHUTE_I2C_SUPERCAP_CAP_STACK_FACTOR_MV;
        if (!is_equal(_state.vcap_total, state.vcap_total)) {
            is_frozen = false;
        }
    }
    if (supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MEAS_VOUT_REG, value)) {
        state.vout = value * AP_PARACHUTE_I2C_SUPERCAP_VIN_VOUT_FACTOR_MV;
        if (!is_equal(_state.vout, state.vout)) {
            is_frozen = false;
        }
    }
    if (supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MEAS_IIN_REG, value)) {
        state.iin = value * AP_PARACHUTE_I2C_SUPERCAP_IIN_ICHG_FACTOR_UV;
        if (!is_equal(_state.iin, state.iin)) {
            is_frozen = false;
        }
    }
    if (supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MEAS_ICHG_REG, value)) {
        state.ichg = value * AP_PARACHUTE_I2C_SUPERCAP_IIN_ICHG_FACTOR_UV;
        if (!is_equal(_state.ichg, state.ichg)) {
            is_frozen = false;
        }
    }
    if (supercap_read_reg(AP_PARACHUTE_I2C_SUPERCAP_MEAS_DTEMP_REG, value)) {
        state.dtemp = AP_PARACHUTE_I2C_SUPERCAP_DTEMP_FACTOR_DEGC(value);
        if (!is_equal(_state.dtemp, state.dtemp)) {
            is_frozen = false;
        }
    }

    const uint64_t now = AP_HAL::micros64();
    if (is_frozen && (_state.frozen_since_us == 0)) {
        _state.frozen_since_us = now;
    }
    if (!is_frozen) {
        _state = state;
        _state.frozen_since_us = 0;
    } 
    if (is_frozen &&
        (((now - _state.frozen_since_us) > MAX_SUPERCAP_REGS_FROZEN_TIME_US) &&
        ((now - _state.last_frozen_error_us) > SUPERCAP_REGS_FROZEN_LOG_FREQ_US))) {
        write_to_log(MAV_SEVERITY_ERROR, "Supercap registers are frozen for more than %u seconds", MAX_SUPERCAP_REGS_FROZEN_TIME_US / 1000 / 1000);
        _state.last_frozen_error_us = now;
    }
    _state.last_updated_us = now;
}

void AP_Parachute_I2C::write_log()
{
    static uint64_t last_log_write_us = 0;

    struct parachute_i2c_state state = _state;
    if (state.last_updated_us > last_log_write_us) {
        struct log_FTS fts_pkt = {
            LOG_PACKET_HEADER_INIT(LOG_FTS_MSG),
            time_us       : state.last_updated_us,
            vcap1         : state.vcap_single[0] / 1000.0f,
            vcap2         : state.vcap_single[1] / 1000.0f,
            vcap3         : state.vcap_single[2] / 1000.0f,
            vcap4         : state.vcap_single[3] / 1000.0f, 
            gpi           : state.gpi / 1000.0f,
            vin           : state.vin,
            vcap          : state.vcap_total,
            vout          : state.vout,
            iin           : state.iin / 1000.0f,
            ichg          : state.ichg / 1000.0f,
            dtemp         : state.dtemp
        };
        AP::logger().WriteBlock(&fts_pkt, sizeof(fts_pkt));
        last_log_write_us = state.last_updated_us;
    }

    int16_t value;
    float vcc = 0;
    if (adc_read_input(adc_input_e::AIN3, value)) {
        vcc = value * ADC_LSB_SIZE * 2.0f;
    }
    struct log_FTS2 fts2_pkt = {
        LOG_PACKET_HEADER_INIT(LOG_FTS2_MSG),
        time_us       : AP_HAL::micros64(),
        pyc           : _pyro_current,
        pysc          : _pyro_short_current,
        busv          : _bus_voltage,
        vcc           : vcc
    };
    AP::logger().WriteBlock(&fts2_pkt, sizeof(fts2_pkt));
}
