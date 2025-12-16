#include <esp_timer.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ads1232.hpp"

static constexpr auto k_tag = "ads1232";

template<unsigned N>
static inline void delay_ns() {
    static_assert(N > 3);
    constexpr unsigned t = N/2.8; // for 360 MHz CPU clock
    #pragma GCC unroll 500
    for(unsigned i = 0; i < t; i++)
         __asm__("nop");
}

Ads1232::Ads1232(const Ads1232Pins&& p) : _pin{p} {
}

void Ads1232::setUp(SPEED s, CHANNEL c, GAIN g)  {
    ESP_ERROR_CHECK(gpio_set_direction(_pin.speed, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(_pin.a0, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(_pin.temp, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(_pin.gain1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(_pin.pdwn, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(_pin.gain0, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(_pin.dout, GPIO_MODE_INPUT));

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << _pin.sclk);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if(setGain(g) != ESP_OK)
        ESP_LOGE(k_tag, "Error setting gain");
    if(setChannel(c) != ESP_OK)
        ESP_LOGE(k_tag, "Error setting channel");
    if(setSpeed(s) != ESP_OK)
        ESP_LOGE(k_tag, "Error setting speed");
    if(powerUp() != ESP_OK)
        ESP_LOGE(k_tag, "Error powering up");
}

void Ads1232::configureReadyInterrupt(gpio_isr_t ih) {
    gpio_set_intr_type(_pin.dout, GPIO_INTR_LOW_LEVEL);
    gpio_isr_handler_add(_pin.dout, ih, NULL);
}


bool Ads1232::isReady() {
    return gpio_get_level(_pin.dout) == 0;
}

esp_err_t Ads1232::powerUp() {
    if(auto r = gpio_set_level(_pin.pdwn, 0); r != ESP_OK) return r;
    esp_rom_delay_us(10);
    if(auto r = gpio_set_level(_pin.pdwn, 1); r != ESP_OK) return r;
    esp_rom_delay_us(26);
    if(auto r = gpio_set_level(_pin.pdwn, 0); r != ESP_OK) return r;
    esp_rom_delay_us(26);
    if(auto r = gpio_set_level(_pin.pdwn, 1); r != ESP_OK) return r;
    delay_ns<100>();
    return gpio_set_level(_pin.sclk, 0);
}

void Ads1232::powerDown() {
    gpio_set_level(_pin.pdwn, 0);
    esp_rom_delay_us(1);
    gpio_set_level(_pin.sclk, 1);
    esp_rom_delay_us(1);
}

esp_err_t Ads1232::setSpeed(SPEED s) {
    _speed = s;
    switch(s) {
        case SPEED::SLOW:
            return gpio_set_level(_pin.speed, 0);
        case SPEED::FAST:
            return gpio_set_level(_pin.speed, 1);
    }

    return ESP_FAIL;
}

esp_err_t Ads1232::setChannel(CHANNEL ch) {
    auto set_gpio = [this](bool temp, bool a0) {
        if(auto r = gpio_set_level(_pin.temp, temp); r != ESP_OK)
            return r;
        if(auto r = gpio_set_level(_pin.a0, a0); r != ESP_OK)
            return r;

        esp_rom_delay_us(50);

        return ESP_OK;
    };

    switch(ch) {
        case CHANNEL::TEMP: return set_gpio(1, 0);
        case CHANNEL::CH0: return set_gpio(0, 0);
        case CHANNEL::CH1: return set_gpio(1, 1);
    }

    return ESP_FAIL;
}

esp_err_t Ads1232::setGain(GAIN g) {
    auto set_gpio = [this](bool g0, bool g1) {
        if(auto r = gpio_set_level(_pin.gain0, g0); r != ESP_OK)
            return r;
        if(auto r = gpio_set_level(_pin.gain1, g1); r != ESP_OK)
            return r;

        return ESP_OK;
    };

    switch(g) {
        case GAIN::GAIN_1: return set_gpio(0, 0);
        case GAIN::GAIN_2: return set_gpio(0, 1);
        case GAIN::GAIN_64: return set_gpio(1, 0);
        case GAIN::GAIN_128: return set_gpio(1, 1);
    }

    return ESP_FAIL;
}

esp_err_t Ads1232::read(long& value, bool cal, bool wait) {
    if(wait) {
        if(auto r = waitReady(cal); r != ESP_OK) return r;
    } else {
        if(!isReady()) [[unlikely]]
            return ESP_ERR_INVALID_STATE;
    }
    value = 0;
    #pragma GCC unroll 24
    for(unsigned i = 0; i < 24; i++) {
        gpio_set_level(_pin.sclk, 1);
        delay_ns<100>();
        value = (value << 1) + gpio_get_level(_pin.dout);
        gpio_set_level(_pin.sclk, 0);
        delay_ns<200>();
    }
    value = (value << 8) / 256;

    if(cal) [[unlikely]] {
        for(unsigned i = 0; i < 2; i++) {
            gpio_set_level(_pin.sclk, 1);
            delay_ns<100>();
            gpio_set_level(_pin.sclk, 0);
            delay_ns<100>();
        }
        if(_speed == SPEED::FAST) {
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(800));
        }
    } else {
        gpio_set_level(_pin.sclk, 1);
         delay_ns<100>();
        gpio_set_level(_pin.sclk, 0);
         delay_ns<100>();
    }

    return ESP_OK;
}

esp_err_t Ads1232::readAverage(float& value, uint8_t times, bool cal, bool wait) {
    if(times == 0) return ESP_ERR_INVALID_ARG;

    long sum = 0;
    long val;

    if(cal) [[unlikely]] read(val, cal, wait);
    for(uint8_t i = 0; i < times; i++) {
        if(auto r = read(val, false, wait); r != ESP_OK) [[unlikely]]
            return r;
        sum += val;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    value = (float)sum / times;
    return ESP_OK;
}

esp_err_t Ads1232::getValue(float& value, uint8_t times, bool cal, bool wait) {
    float val{};
    if(auto r = readAverage(val, times, cal, wait); r != ESP_OK) [[unlikely]] return r;
    value = val - _offset;
    return ESP_OK;
}

esp_err_t Ads1232::getUnits(float& value, uint8_t times, bool cal, bool wait) {
    if(_scale == 0) [[unlikely]]  return ESP_ERR_INVALID_SIZE;

    float v = 0;
    if(auto r = getValue(v, times, cal, wait); r != ESP_OK) [[unlikely]] return r;
    value = v / _scale;
    return ESP_OK;
}

esp_err_t Ads1232::tare(uint8_t times, bool cal) {
    float sum = 0;
    ESP_RETURN_ON_ERROR(readAverage(sum, times, cal), k_tag, "Can not read");
    setOffset(sum);
    return ESP_OK;
}

void Ads1232::setScale(float s) {
    _scale = s;
}

float Ads1232::getScale() const {
    return _scale;
}

void Ads1232::setOffset(float o) {
    _offset = o;
}

float Ads1232::getOffset() const {
    return _offset;
}

// private methods

esp_err_t Ads1232::waitReady(bool cal) {
    unsigned timeout;

    if(cal) {
        timeout = (_speed == SPEED::FAST) ? 150 : 850;
    } else {
        timeout = (_speed == SPEED::FAST) ? 20 : 150;
    }
    timeout += 600;

    unsigned long start = esp_timer_get_time() / 1000;
    while(gpio_get_level(_pin.dout) != 1) {
        if((esp_timer_get_time() / 1000 - start) > timeout) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    start = esp_timer_get_time() / 1000;
    while(gpio_get_level(_pin.dout) != 0) {
        if((esp_timer_get_time() / 1000 - start) > timeout) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_OK;
}

