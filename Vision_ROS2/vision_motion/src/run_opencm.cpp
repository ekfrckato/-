#include "vision_motion/run_opencm.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#include <iostream>
#include <cmath>
#include <iomanip>

DynamixelController::DynamixelController() : Node("dynamixel_controller") {
    declare_parameter("WO1", 0.54);
    declare_parameter("WO2", 0.04);
    declare_parameter("WO3", 0.29);
    declare_parameter("AW", 0.40);
    get_parameter("WO1", wheelOffset1_);
    get_parameter("WO2", wheelOffset2_);
    get_parameter("WO3", wheelOffset3_);
    get_parameter("AW", axleWidth_);
    
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&DynamixelController::twist_cb ,this, _1));
    timer_ = this->create_wall_timer(10ms, std::bind(&DynamixelController::timer_cb, this));
    std::fill(motPos_, motPos_ + 6, 3073);
}
void DynamixelController::twist_cb(const geometry_msgs::msg::Twist msg) {
    double radius = std::abs(msg.linear.x / msg.angular.z);
    
    if (msg.angular.z < -0.01 && radius > 0.01) {
        motPos_[0] = 3073 + (std::atan2(wheelOffset1_, radius + (axleWidth_ / 2)) * 3073 / PI);
        motPos_[1] = 3073 + (std::atan2(wheelOffset1_, radius - (axleWidth_ / 2)) * 3073 / PI);
        motPos_[2] = 3073 + (std::atan2(wheelOffset2_, radius + (axleWidth_ / 2)) * 3073 / PI);
        motPos_[3] = 3073 + (std::atan2(wheelOffset2_, radius - (axleWidth_ / 2)) * 3073 / PI);
        motPos_[4] = 3073 - (std::atan2(wheelOffset3_, radius + (axleWidth_ / 2)) * 3073 / PI);
        motPos_[5] = 3073 - (std::atan2(wheelOffset3_, radius - (axleWidth_ / 2)) * 3073 / PI);
    }
    else if (msg.angular.z > 0.01 && radius > 0.01) {
        motPos_[0] = 3073 - (std::atan2(wheelOffset1_, radius - (axleWidth_ / 2)) * 3073 / PI);
        motPos_[1] = 3073 - (std::atan2(wheelOffset1_, radius + (axleWidth_ / 2)) * 3073 / PI);
        motPos_[2] = 3073 - (std::atan2(wheelOffset2_, radius - (axleWidth_ / 2)) * 3073 / PI);
        motPos_[3] = 3073 - (std::atan2(wheelOffset2_, radius + (axleWidth_ / 2)) * 3073 / PI);
        motPos_[4] = 3073 + (std::atan2(wheelOffset3_, radius - (axleWidth_ / 2)) * 3073 / PI);
        motPos_[5] = 3073 + (std::atan2(wheelOffset3_, radius + (axleWidth_ / 2)) * 3073 / PI);
    }
    else if (std::abs(msg.angular.z) > 0.03 && radius < 0.01) {
        std::cout << "for zero_radious turn  " << std::endl;
        double calculation = (std::atan2(wheelOffset1_ + wheelOffset3_, axleWidth_) * 3073 / M_PI);
        std::cout << "Calculation result: " << calculation << std::endl;
        motPos_[0] = 3073 + (std::atan2(wheelOffset1_ + wheelOffset3_, axleWidth_)* 3073 / PI);
        motPos_[1] = 3073 - (std::atan2(wheelOffset1_ + wheelOffset3_, axleWidth_)* 3073 / PI);
        motPos_[2] = 3073 + (std::atan2(wheelOffset2_ , axleWidth_)* 3073 / PI);
        motPos_[3] = 3073 - (std::atan2(wheelOffset2_ , axleWidth_)* 3073 / PI);
        motPos_[4] = 3073 - (std::atan2(wheelOffset1_ + wheelOffset3_, axleWidth_)* 3073 / PI);
        motPos_[5] = 3073 + (std::atan2(wheelOffset1_ + wheelOffset3_, axleWidth_)* 3073 / PI);
    }   
    else {
        motPos_[0] = 3073;
        motPos_[1] = 3073;
        motPos_[2] = 3073;
        motPos_[3] = 3073;
        motPos_[4] = 3073;
        motPos_[5] = 3073;
    }
}
void DynamixelController::timer_cb() {
    writePosition(motPos_);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();
    return 0;
}
