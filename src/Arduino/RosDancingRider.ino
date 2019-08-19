#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include <WiFi.h>

#include "PwmServoDriver.hpp"

// PWM出力ESC ID
static const int kMotorId = PwmServoDriver::kSvr3;

// PWM出力サーボ ID
static const int kSteerId = PwmServoDriver::kSvr0;

// 右LED出力ポート
static const int kLedRight = 14;

// 左LED出力ポート
static const int kLedLeft = 25;

// サーボドライブ
PwmServoDriver servo;

// Wi-Fi アクセスポイント名
const char Ssid[] = "hogehogehoge";

// Wi-Fi 暗号化キー
const char SsidKey[] = "hogefugahoge";

// ROSサーバ出力ポート番号
const uint16_t ServerPort = 11411;

// ROSサーバIPアドレス
IPAddress server(192,168,1.1);

///ここからサンプルコピペ >>>
/// 参照先: https://qiita.com/nnn112358/items/1cd4517ea2faa57e9c13
WiFiClient client;

class WiFiHardware {
  public:
  WiFiHardware() {};

  void init() {
    client.connect(server, ServerPort);
  }

  int read() {
    return client.read();
  }

  void write(uint8_t* data, int length) {
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  unsigned long time() {
     return millis();
  }
};

///ここまでサンプルコピペ <<<

uint64_t last_recv;
bool is_force_stop = false;
ros::NodeHandle_<WiFiHardware>  nh;

// ジョイスティックサブスクライブ処理
void messageSitckValue(const std_msgs::Float32MultiArray& msg) {

    // モータの速度を抑制
    float speed = msg.data[0];
    if (fabs(speed) < 0.2) speed = speed * 0.2f;
    else speed = (speed < 0 ? -1 : 1) * ((fabs(speed)-0.2) * 0.3 + 0.04);

    servo.Drive(kMotorId, speed);
    float steer = msg.data[1];

    servo.Drive(kSteerId, steer);

    // 矯正停止フラグ下げる
    is_force_stop = false;

    // 受信時刻取得
    last_recv = millis();
}

ros::Subscriber<std_msgs::Float32MultiArray> subJoy("joy_pub", &messageSitckValue);

// ヘッドライト出力初期化
static void InitHeadLed() {
    pinMode(kLedLeft, OUTPUT);
    pinMode(kLedRight, OUTPUT);
}

// ヘッドライト消灯
static void HeadLedOff() {
    digitalWrite(kLedLeft, LOW);
    digitalWrite(kLedRight, LOW);
}

// ヘッドライト点灯
static void HeadLedOn() {
    digitalWrite(kLedLeft, HIGH);
    digitalWrite(kLedRight, HIGH);
}

static void InitServo() {
    servo.Initialize();
    servo.SetFlip(kMotorId, true);
    servo.SetFlip(kSteerId, true);
    servo.SetTrim(kSteerId, 0.15);
    servo.Drive(kMotorId, 0);
    servo.Drive(kSteerId, 0);
    delay(3000);
}

// ROSノード初期化
void InitRos() {
    last_recv = millis();

    // Wi-Fiを先に切断しないとうまくいかない
    WiFi.disconnect(true);

    // Wi-Fi初期化
    WiFi.begin(Ssid, SsidKey);

    // 接続完了待ち
    while (WiFi.status() != WL_CONNECTED) delay(100);

    // ノード初期化
    nh.initNode();
    nh.subscribe(subJoy);
}

void setup() {
    InitHeadLed();
    HeadLedOff();

    InitServo();
    InitRos();

    HeadLedOn();
}

void loop() {
    nh.spinOnce();
    //delay(1);
    uint64_t c = millis();
    if (!is_force_stop && (c - last_recv) > 1000) {
        is_force_stop = true;
        servo.Drive(kMotorId, 0);
    }
}