#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vision_motion/motor_command.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define PI 3.1415926535897932

class DDSM_Twist : public rclcpp::Node {
public:
    DDSM_Twist() : Node("DDSM115"), port1(0), port2(2), port3(4), port4(6){
        declare_parameter("WO1", 0.54);
        declare_parameter("WO2", 0.04);
        declare_parameter("WO3", 0.29);
        declare_parameter("AW", 0.40);
        declare_parameter("WD", 0.1);
        get_parameter("WO1", wheelOffset1_);
        get_parameter("WO2", wheelOffset2_);
        get_parameter("WO3", wheelOffset3_);
        get_parameter("AW", axleWidth_);
        get_parameter("WD", wheelDiameter_);

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&DDSM_Twist::twist_cb, this, _1));
        timer_ = this->create_wall_timer(50ms, std::bind(&DDSM_Twist::timer_cb, this));
    }
private:
    void timer_cb() {
        port1.SET_VELOCITY(1, motor_rpm_[0]);
        port2.SET_VELOCITY(1, motor_rpm_[1]);
        if (timer == 0) {
            port3.SET_VELOCITY(1, motor_rpm_[2]);
            port4.SET_VELOCITY(1, motor_rpm_[3]);
            timer = 1;
        }
        else {
            port3.SET_VELOCITY(2, motor_rpm_[4]);
            port4.SET_VELOCITY(2, motor_rpm_[5]);
            timer = 0;
        }
    }

    void twist_cb(const geometry_msgs::msg::Twist msg) {
        double v = msg.linear.x;
        double omega = msg.angular.z;
        auto radius = std::abs(v / omega);
        int ZeroRPM = 3

        wheel_omega[0] = v * sqrt(pow((radius - axleWidth_ / 2), 2) + pow((wheelOffset1_ / 2), 2)) / radius / wheelDiameter_ * 2;
        wheel_omega[1] = v * sqrt(pow((radius + axleWidth_ / 2), 2) + pow((wheelOffset1_ / 2), 2)) / radius / wheelDiameter_ * 2;
        wheel_omega[2] = v * sqrt(pow((radius - axleWidth_ / 2), 2) + pow((wheelOffset2_ / 2), 2)) / radius / wheelDiameter_ * 2;
        wheel_omega[3] = v * sqrt(pow((radius + axleWidth_ / 2), 2) + pow((wheelOffset2_ / 2), 2)) / radius / wheelDiameter_ * 2;
        wheel_omega[4] = v * sqrt(pow((radius - axleWidth_ / 2), 2) + pow((wheelOffset3_ / 2), 2)) / radius / wheelDiameter_ * 2;
        wheel_omega[5] = v * sqrt(pow((radius + axleWidth_ / 2), 2) + pow((wheelOffset3_ / 2), 2)) / radius / wheelDiameter_ * 2;

        if (std::abs(omega) > 0.25 && radius == 0.) {
            if (omega > 0.01) {
                motor_rpm_[0] = ZeroRPM
                motor_rpm_[1] = ZeroRPM
                motor_rpm_[2] = (ZeroRPM * wheelOffset2_ / wheelOffset1_)
                motor_rpm_[3] = (ZeroRPM * wheelOffset2_ / wheelOffset1_)
                motor_rpm_[4] = ZeroRPM
                motor_rpm_[5] = ZeroRPM
                }
            else if (omega < -0.01) {
                motor_rpm_[0] = -ZeroRPM
                motor_rpm_[1] = -ZeroRPM
                motor_rpm_[2] = -(ZeroRPM * wheelOffset2_ / wheelOffset1_)
                motor_rpm_[3] = -(ZeroRPM * wheelOffset2_ / wheelOffset1_)
                motor_rpm_[4] = -ZeroRPM
                motor_rpm_[5] = -ZeroRPM
                    }
            }      
        else if (omega < -0.01 && radius > 0.01) {
            motor_rpm_[0] = wheel_omega[1] * 30 / PI;
            motor_rpm_[1] = -1 * wheel_omega[0] * 30 / PI;
            motor_rpm_[2] = wheel_omega[3] * 30 / PI;
            motor_rpm_[3] = -1 * wheel_omega[2] * 30 / PI;
            motor_rpm_[4] = wheel_omega[5] * 30 / PI;
            motor_rpm_[5] = -1 * wheel_omega[4] * 30 / PI;
        }
        else if(omega > 0.01 && radius > 0.01) {
            motor_rpm_[0] = wheel_omega[0] * 30 / PI;
            motor_rpm_[1] = -1 * wheel_omega[1] * 30 / PI;
            motor_rpm_[2] = wheel_omega[2] * 30 / PI;
            motor_rpm_[3] = -1 * wheel_omega[3] * 30 / PI;
            motor_rpm_[4] = wheel_omega[4] * 30 / PI;
            motor_rpm_[5] = -1 * wheel_omega[5] * 30 / PI;
        }
        else {
            motor_rpm_[0] = (60 * v) / (wheelDiameter_ * PI);
            motor_rpm_[1] = -1 * (60 * v) / (wheelDiameter_ * PI);
            motor_rpm_[2] = (60 * v) / (wheelDiameter_ * PI);
            motor_rpm_[3] = -1 * (60 * v) / (wheelDiameter_ * PI);
            motor_rpm_[4] = (60 * v) / (wheelDiameter_ * PI);
            motor_rpm_[5] = -1 * (60 * v) / (wheelDiameter_ * PI);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    double wheel_omega[6];
    double motor_rpm_[6] = {0.0, }; // LF, RF, LM, RM, LR, RR
    MOTOR_COMMAND port1, port2, port3, port4;
    double wheelOffset1_, wheelOffset2_, wheelOffset3_,axleWidth_, wheelDiameter_;
    int timer = 0; // 0: middle wheel, 1: rear wheel
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DDSM_Twist>());
    rclcpp::shutdown();
    return 0;
}
