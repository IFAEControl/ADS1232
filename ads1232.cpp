#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ads1232.hpp"

Ads1232::Ads1232(const Ads1232Pins&& p) : _pin{p} {
}

void Ads1232::setUp(SPEED s, CHANNEL c, GAIN g)  {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << _pin.sclk);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_direction(_pin.dout, GPIO_MODE_INPUT);
    gpio_set_direction(_pin.pdwn, GPIO_MODE_OUTPUT);
    gpio_set_direction(_pin.speed, GPIO_MODE_OUTPUT);
    gpio_set_direction(_pin.a0, GPIO_MODE_OUTPUT);
    gpio_set_direction(_pin.temp, GPIO_MODE_OUTPUT);
    gpio_set_direction(_pin.gain0, GPIO_MODE_OUTPUT);
    gpio_set_direction(_pin.gain1, GPIO_MODE_OUTPUT);

    setGain(g);
    setChannel(c);
    setSpeed(s);
    powerUp();
}

bool Ads1232::isReady() {
    return gpio_get_level(_pin.dout) == 0;
}

void Ads1232::powerUp() {
    gpio_set_level(_pin.pdwn, 0);
    esp_rom_delay_us(20);
    gpio_set_level(_pin.pdwn, 1);
    esp_rom_delay_us(20);
    gpio_set_level(_pin.pdwn, 0);
    esp_rom_delay_us(30);
    gpio_set_level(_pin.pdwn, 1);
    esp_rom_delay_us(1);
    gpio_set_level(_pin.sclk, 0);
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
        if(auto r = gpio_set_level(_pin.temp, a0); r != ESP_OK)
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

ERROR_t Ads1232::read(long& value, bool Calibrating) {
    int i = 0;
    unsigned long start;
    unsigned int waitingTime;
    value = 0;

    if(Calibrating) {
        waitingTime = (_speed == SPEED::FAST) ? 150 : 850;
    } else {
        waitingTime = (_speed == SPEED::FAST) ? 20 : 150;
    }
    waitingTime += 600;

    start = esp_timer_get_time() / 1000;
    while(gpio_get_level(_pin.dout) != 1) {
        if((esp_timer_get_time() / 1000 - start) > waitingTime) return TIMEOUT_HIGH;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    start = esp_timer_get_time() / 1000;
    while(gpio_get_level(_pin.dout) != 0) {
        if((esp_timer_get_time() / 1000 - start) > waitingTime) return TIMEOUT_LOW;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    for(i = 23; i >= 0; i--) {
        gpio_set_level(_pin.sclk, 1);
        esp_rom_delay_us(1);
        value = (value << 1) + gpio_get_level(_pin.dout);
        gpio_set_level(_pin.sclk, 0);
        esp_rom_delay_us(1);
    }

    if(Calibrating) {
        for(i = 1; i >= 0; i--) {
            gpio_set_level(_pin.sclk, 1);
            esp_rom_delay_us(1);
            gpio_set_level(_pin.sclk, 0);
            esp_rom_delay_us(1);
        }
        if(_speed == SPEED::FAST) {
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(800));
        }
    }

    value = (value << 8) / 256;

    if(!Calibrating) {
        gpio_set_level(_pin.sclk, 1);
        esp_rom_delay_us(1);
        gpio_set_level(_pin.sclk, 0);
        esp_rom_delay_us(1);
    }
    return NoERROR;
}

ERROR_t Ads1232::read_average(float& value, uint8_t times, bool Calibrating) {
    long sum = 0;
    ERROR_t err;
    long val;

    if(Calibrating) read(val, true);
    for(uint8_t i = 0; i < times; i++) {
        err = read(val, false);
        if(err != NoERROR) return err;
        sum += val;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if(times == 0) return DIVIDED_by_ZERO;
    value = (float)sum / times;
    return NoERROR;
}

ERROR_t Ads1232::get_value(float& value, uint8_t times, bool Calibrating) {
    float val = 0;
    ERROR_t err;
    err = read_average(val, times, Calibrating);
    if(err != NoERROR) return err;
    value = val - _offset;
    return NoERROR;
}

ERROR_t Ads1232::get_units(float& value, uint8_t times, bool Calibrating) {
    float val = 0;
    ERROR_t err;
    err = get_value(val, times, Calibrating);
    if(err != NoERROR) return err;
    if(_scale == 0) return DIVIDED_by_ZERO;
    value = val / _scale;
    return NoERROR;
}

ERROR_t Ads1232::tare(uint8_t times, bool Calibrating) {
    ERROR_t err;
    float sum = 0;
    err = read_average(sum, times, Calibrating);
    if(err != NoERROR) return err;
    setOffset(sum);
    return NoERROR;
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
