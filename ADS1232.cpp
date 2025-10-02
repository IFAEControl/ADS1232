#include <thread>
#include <chrono>

#include "ADS1232.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

using namespace std::chrono_literals;

ADS1232::ADS1232() {
}

ADS1232::~ADS1232() {
}

void ADS1232::begin(gpio_num_t pin_DOUT, gpio_num_t pin_SCLK, gpio_num_t pin_PDWN, gpio_num_t pin_SPEED, Speed speed) {
    _pin_DOUT = pin_DOUT;
    _pin_SCLK = pin_SCLK;
    _pin_PDWN = pin_PDWN;
    _pin_SPEED = pin_SPEED;
    
/*    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << _pin_DOUT);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    */

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << _pin_SCLK);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_direction(_pin_DOUT, GPIO_MODE_INPUT);
    gpio_set_direction(_pin_PDWN, GPIO_MODE_OUTPUT);
    gpio_set_direction(_pin_SPEED, GPIO_MODE_OUTPUT);

    setSpeed(speed);
    power_up();
}

bool ADS1232::is_ready() {
    return gpio_get_level(_pin_DOUT) == 0;
}

void ADS1232::power_up() {
    gpio_set_level(_pin_PDWN, 0);
    esp_rom_delay_us(20);
    gpio_set_level(_pin_PDWN, 1);
    esp_rom_delay_us(20);
    gpio_set_level(_pin_PDWN, 0);
    esp_rom_delay_us(30);
    gpio_set_level(_pin_PDWN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(_pin_SCLK, 0);
}

void ADS1232::power_down() {
    gpio_set_level(_pin_PDWN, 0);
    esp_rom_delay_us(1);
    gpio_set_level(_pin_SCLK, 1);
    esp_rom_delay_us(1);
}

void ADS1232::setSpeed(Speed speed) {
    _speed = speed;
    switch(speed) {
        case SLOW:
            gpio_set_level(_pin_SPEED, 0);
            break;
        case FAST:
            gpio_set_level(_pin_SPEED, 1);
            break;
    }
    esp_rom_delay_us(1);
}

ERROR_t ADS1232::read(long& value, bool Calibrating) {
    int i = 0;
    unsigned long start;
    unsigned int waitingTime;
    value = 0;

    if(Calibrating) {
        waitingTime = (_speed == FAST) ? 150 : 850;
    } else {
        waitingTime = (_speed == FAST) ? 20 : 150;
    }
    waitingTime += 600;

    start = esp_timer_get_time() / 1000;
    while(gpio_get_level(_pin_DOUT) != 1) {
        if((esp_timer_get_time() / 1000 - start) > waitingTime) return TIMEOUT_HIGH;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    start = esp_timer_get_time() / 1000;
    while(gpio_get_level(_pin_DOUT) != 0) {
        if((esp_timer_get_time() / 1000 - start) > waitingTime) return TIMEOUT_LOW;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    for(i = 23; i >= 0; i--) {
        gpio_set_level(_pin_SCLK, 1);
        esp_rom_delay_us(1);
        value = (value << 1) + gpio_get_level(_pin_DOUT);
        gpio_set_level(_pin_SCLK, 0);
        esp_rom_delay_us(1);
    }

    if(Calibrating) {
        for(i = 1; i >= 0; i--) {
            gpio_set_level(_pin_SCLK, 1);
            esp_rom_delay_us(1);
            gpio_set_level(_pin_SCLK, 0);
            esp_rom_delay_us(1);
        }
        if(_speed == FAST) {
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(800));
        }
    }

    value = (value << 8) / 256;

    if(!Calibrating) {
        gpio_set_level(_pin_SCLK, 1);
        esp_rom_delay_us(1);
        gpio_set_level(_pin_SCLK, 0);
        esp_rom_delay_us(1);
    }
    return NoERROR;
}

ERROR_t ADS1232::read_average(float& value, uint8_t times, bool Calibrating) {
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

ERROR_t ADS1232::get_value(float& value, uint8_t times, bool Calibrating) {
    float val = 0;
    ERROR_t err;
    err = read_average(val, times, Calibrating);
    if(err != NoERROR) return err;
    value = val - OFFSET;
    return NoERROR;
}

ERROR_t ADS1232::get_units(float& value, uint8_t times, bool Calibrating) {
    float val = 0;
    ERROR_t err;
    err = get_value(val, times, Calibrating);
    if(err != NoERROR) return err;
    if(SCALE == 0) return DIVIDED_by_ZERO;
    value = val / SCALE;
    return NoERROR;
}

ERROR_t ADS1232::tare(uint8_t times, bool Calibrating) {
    ERROR_t err;
    float sum = 0;
    err = read_average(sum, times, Calibrating);
    if(err != NoERROR) return err;
    set_offset(sum);
    return NoERROR;
}

void ADS1232::set_scale(float scale) {
    SCALE = scale;
}

float ADS1232::get_scale() {
    return SCALE;
}

void ADS1232::set_offset(float offset) {
    OFFSET = offset;
}

float ADS1232::get_offset() {
    return OFFSET;
}
