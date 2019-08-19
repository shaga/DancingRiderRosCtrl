#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>

// ジョイスティック操作送信クラス
class JoyToRider {
public:
    JoyToRider();

private:
    const int kStickSpeed = 1;
    const int kStickSteer = 3;

    enum PubStickMsg {
        kMsgSpeed,
        kMsgSteer,
        kMsgLength,
    };

    void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void TimerCallback(const ros::TimerEvent&);
    void PublishMessage();

    ros::NodeHandle nh_;
    ros::Publisher joy_pub_;
    ros::Subscriber joy_sub_;
    ros::Timer joy_timer_;

    _Float32 stick_msg_[kMsgLength];
};

JoyToRider::JoyToRider() {
    // ゲームパッドデータ送信パブリッシャ登録
    joy_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("joy_pub", 1);
    // ゲームパッドデータ受信サブスクライバ登録
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyToRider::JoyCallback, this);
    // 定期送信タイマ生成
    joy_timer_ = nh_.createTimer(ros::Duration(0.5), &JoyToRider::TimerCallback, this);
}

// ゲームパッドデータパブリッシュ
void JoyToRider::PublishMessage() {
    std_msgs::Float32MultiArray msg;
    msg.data.resize(kMsgLength);
    for (int i = 0; i < kMsgLength; i++) {
        msg.data[i] = stick_msg_[i];
    }

    joy_pub_.publish(msg);
}

// ゲームパッドサブスクライバコールバック
void JoyToRider::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    stick_msg_[kMsgSpeed] = joy->axes[kStickSpeed];
    stick_msg_[kMsgSteer] = joy->axes[kStickSteer];

    PublishMessage();
}

// 定期送信タイマコールバック
void JoyToRider::TimerCallback(const ros::TimerEvent&) {
    PublishMessage();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_dancing_rider");
    JoyToRider rider;
    ros::spin();
    return 0;
}
