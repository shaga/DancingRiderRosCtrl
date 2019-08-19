#ifndef __PWM_SERVO_DRIVER_HPP__
#define __PWM_SERVO_DRIVER_HPP__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

// サーボ出力設定
struct ServoIo {
    ledc_timer_t    pwm_timer;      // 利用タイマID
    ledc_channel_t  pwm_channel;    // PWMチャンネル
    gpio_num_t      pwm_io;         // PWM出力ポート
};

// PWM出力クラス
class PwmServoDriver {
public:
    // サーボID
    enum ServoName {
        kSvr0,
        kSvr1,
        kSvr2,
        kSvr3,
        kServoCount,
    };

    PwmServoDriver();

    // 初期化
    esp_err_t Initialize();

    // 駆動
    esp_err_t Drive(int ch, double pos);

    // サーボ反転設定
    void SetFlip(int ch, bool is_flip);

    // サーボトリム設定
    void SetTrim(int ch, int16_t trim);
private:
    // サーボタイマ設定
    enum UseTimeChannel {
        kTimerSvr01,
        kTimerSvr23,
        kTimerCount,
    };

    // PWM週蓮
    static const uint32_t kPwmFrequency;

    // 標準サーボニュートラル値
    static const uint16_t kDefaultServoCenter;

    // サーボ駆動範囲
    static const uint16_t kServoRange;

    // サーボ出力最小値
    static const uint16_t kServoPosMin;

    // サーバ出力最大値
    static const uint16_t kServoPosMax;

    // 各サーボ利用タイマチャンネル
    static const ledc_timer_t kUseTimer[kTimerCount];

    // サーボ設定
    static const ServoIo kServoInfo[kServoCount]; 

    // タイマ初期化
    esp_err_t InitTimers();

    // PWM出力初期化
    esp_err_t InitPwm();

    bool is_flip_[kServoCount];
    int16_t trim_[kServoCount];
};

#endif //__PWM_SERVO_DRIVER_HPP__