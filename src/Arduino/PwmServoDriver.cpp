#include <string.h>
#include "PwmServoDriver.hpp"

const uint32_t PwmServoDriver::kPwmFrequency = 50;

const uint16_t PwmServoDriver::kDefaultServoCenter = 4915;

const uint16_t PwmServoDriver::kServoRange = 1638;

const uint16_t PwmServoDriver::kServoPosMin = 1638;
const uint16_t PwmServoDriver::kServoPosMax = 8192;

const ledc_timer_t PwmServoDriver::kUseTimer[] ={LEDC_TIMER_0, LEDC_TIMER_1};

const ServoIo PwmServoDriver::kServoInfo[] = {
    {kUseTimer[kTimerSvr01], LEDC_CHANNEL_0, GPIO_NUM_5},
    {kUseTimer[kTimerSvr01], LEDC_CHANNEL_1, GPIO_NUM_17},
    {kUseTimer[kTimerSvr23], LEDC_CHANNEL_2, GPIO_NUM_16},
    {kUseTimer[kTimerSvr23], LEDC_CHANNEL_3, GPIO_NUM_4}
};


PwmServoDriver::PwmServoDriver() {
    memset(trim_, 0, sizeof(trim_));
    memset(is_flip_, 0, sizeof(is_flip_));
}

esp_err_t PwmServoDriver::Initialize() {
    esp_err_t ret = ESP_OK;

    if (ret == ESP_OK) {
        ret = InitTimers();
    }

    if (ret == ESP_OK) {
        ret = InitPwm();
    }

    return ret;
}

void PwmServoDriver::SetFlip(int ch, bool is_flip) {
    if (ch < 0 || kServoCount <= ch) {
        return;
    }

    is_flip_[ch] = is_flip;    
}

void PwmServoDriver::SetTrim(int ch, int16_t trim) {
    if (ch < 0 || kServoCount <= ch) {
        return;
    }

    trim_[ch] = trim;
}

esp_err_t PwmServoDriver::Drive(int ch, double pos) {
    if (ch < 0 || kServoCount <= ch) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    if (pos < -1.0) pos = -1.0;
    else if (pos > 1.0) pos = 1.0;

    uint16_t duty = kDefaultServoCenter + (kServoRange * pos + trim_[ch]) * (is_flip_[ch] ? -1 : 1);

    ret = ledc_set_duty(LEDC_HIGH_SPEED_MODE, kServoInfo[ch].pwm_channel, duty);

    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_HIGH_SPEED_MODE, kServoInfo[ch].pwm_channel);
    }

    return ret;
}

esp_err_t PwmServoDriver::InitTimers() {
    esp_err_t ret = ESP_OK;

    ledc_timer_config_t timer_config;
    timer_config.duty_resolution = LEDC_TIMER_16_BIT;
    timer_config.freq_hz = kPwmFrequency;
    timer_config.speed_mode = LEDC_HIGH_SPEED_MODE;

    for (int i = 0; i < kTimerCount && ret == ESP_OK; i++) {
        timer_config.timer_num = kUseTimer[i];
        ret = ledc_timer_config(&timer_config);
    }

    return ret;
}

esp_err_t PwmServoDriver::InitPwm() {
    esp_err_t ret = ESP_OK;

    ledc_channel_config_t pwm_config;
    pwm_config.duty = 0;
    pwm_config.hpoint = 0;
    pwm_config.speed_mode = LEDC_HIGH_SPEED_MODE;
    pwm_config.intr_type = LEDC_INTR_DISABLE;

    for (int i = 0; i < kServoCount && ret == ESP_OK; i++) {
        pwm_config.channel = kServoInfo[i].pwm_channel;
        pwm_config.timer_sel = kServoInfo[i].pwm_timer;
        pwm_config.gpio_num = kServoInfo[i].pwm_io;

        ret = ledc_channel_config(&pwm_config);
    }

    return ret;
}