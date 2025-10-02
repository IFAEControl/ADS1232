#pragma once

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum ERROR_t {
    NoERROR,
    TIMEOUT_HIGH,     // Timeout waiting for HIGH
    TIMEOUT_LOW,      // Timeout waiting for LOW
    WOULD_BLOCK,      // weight not measured, measuring takes too long
    STABLE_TIMEOUT,   // weight not stable within timeout
    DIVIDED_by_ZERO
};

enum Speed {
    SLOW = 0,
    FAST
};

class ADS1232 {
public:
    ADS1232();
    virtual ~ADS1232();

    void begin(gpio_num_t pin_DOUT, gpio_num_t pin_SCLK, gpio_num_t pin_PDWN, gpio_num_t pin_SPEED, Speed speed = SLOW);
    bool is_ready();
    void setSpeed(Speed speed);
    ERROR_t read(long& value, bool Calibrating = false);
    ERROR_t read_average(float& value, uint8_t times = 10, bool Calibrating = false);
    ERROR_t get_value(float& value, uint8_t times = 1, bool Calibrating = false);
    ERROR_t get_units(float& value, uint8_t times = 1, bool Calibrating = false);
    ERROR_t tare(uint8_t times = 10, bool Calibrating = false);
    void set_scale(float scale = 1.f);
    float get_scale();
    void set_offset(float offset = 0);
    float get_offset();
    void power_down();
    void power_up();

private:
    gpio_num_t _pin_DOUT;
    gpio_num_t _pin_SCLK;
    gpio_num_t _pin_PDWN;
    gpio_num_t _pin_SPEED;

    float OFFSET = 0;
    float SCALE = 1;
    Speed _speed;
};
