#pragma once

#include <driver/gpio.h>
#include <esp_timer.h>

enum ERROR_t {
    NoERROR,
    TIMEOUT_HIGH,     // Timeout waiting for HIGH
    TIMEOUT_LOW,      // Timeout waiting for LOW
    WOULD_BLOCK,      // weight not measured, measuring takes too long
    STABLE_TIMEOUT,   // weight not stable within timeout
    DIVIDED_by_ZERO
};

enum class SPEED : uint8_t {
    SLOW,
    FAST
};

enum class CHANNEL : uint8_t {
    CH0,
    CH1,
    TEMP,
};

enum class GAIN : uint8_t {
    GAIN_1,
    GAIN_2,
    GAIN_64,
    GAIN_128
};

struct Ads1232Pins {
    gpio_num_t dout;
    gpio_num_t sclk;
    gpio_num_t pdwn;
    gpio_num_t speed;
    gpio_num_t a0;
    gpio_num_t temp;
    gpio_num_t gain0;
    gpio_num_t gain1;
};

class Ads1232 {
public:
    Ads1232(const Ads1232Pins&& p);
    ~Ads1232() = default;

    void setUp(SPEED s = SPEED::SLOW, CHANNEL c = CHANNEL::CH0, GAIN g = GAIN::GAIN_1);
    bool isReady();
    esp_err_t setSpeed(SPEED);
    esp_err_t setChannel(CHANNEL);
    esp_err_t setGain(GAIN);
    ERROR_t read(long& value, bool Calibrating = false);
    ERROR_t read_average(float& value, uint8_t times = 10, bool Calibrating = false);
    ERROR_t get_value(float& value, uint8_t times = 1, bool Calibrating = false);
    ERROR_t get_units(float& value, uint8_t times = 1, bool Calibrating = false);
    ERROR_t tare(uint8_t times = 10, bool Calibrating = false);
    void setScale(float);
    float getScale() const;
    void setOffset(float);
    float getOffset() const;
    void powerDown();
    void powerUp();

private:
    const Ads1232Pins _pin;

    SPEED _speed{SPEED::SLOW};
    CHANNEL _channel{CHANNEL::CH0};
    float _offset{0};
    float _scale{1};
};
