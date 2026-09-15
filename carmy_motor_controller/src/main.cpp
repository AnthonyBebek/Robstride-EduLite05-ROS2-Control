#include "motor_ros2/motor_cfg.h"
#include "carmy_motor_controller/msg/motor_motion_control.hpp"
#include "carmy_motor_controller/msg/motor_feedback.hpp"
#include "stdint.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <unistd.h>
#include <vector>

using MotorMotionControl = carmy_motor_controller::msg::MotorMotionControl;
using MotorFeedback = carmy_motor_controller::msg::MotorFeedback;

class MotorControlSample : public rclcpp::Node {
public:
  MotorControlSample()
      : rclcpp::Node("motor_control_set_node"),
        motor(RobStrideMotor("can0", 0xFF, 0x03, 0)) {

    motor.Get_RobStrite_Motor_parameter(0x7005);
    usleep(1000);
    motor.enable_motor();
    usleep(1000);

    motion_sub_ = this->create_subscription<MotorMotionControl>(
        "motion_control", 10,
        std::bind(&MotorControlSample::motion_callback, this,
                  std::placeholders::_1));

    feedback_pub_ = this->create_publisher<MotorFeedback>("motor_feedback", 10);

    target_position_ = 0.0;
    target_velocity_ = 0.0;
    target_kp_ = 1.0;
    target_kd_ = 0.1;
    target_torque_ = 0.0;

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MotorControlSample::hold_position, this));
  }

  ~MotorControlSample() {
    motor.Disenable_Motor(0);
  }

  void motion_callback(const MotorMotionControl::SharedPtr msg) {
    target_position_ = msg->position;
    target_velocity_ = msg->velocity;
    target_kp_ = msg->kp;
    target_kd_ = msg->kd;
    target_torque_ = msg->torque;

    apply_target();

    RCLCPP_INFO(this->get_logger(),
                "motion cmd: pos=%.3f vel=%.3f kp=%.3f kd=%.3f torque=%.3f",
                target_position_, target_velocity_, target_kp_, target_kd_,
                target_torque_);
  }

  void apply_target() {
    auto [position_feedback, velocity_feedback, torque, temperature] =
        motor.send_motion_command(target_torque_, target_position_,
                                 target_velocity_, target_kp_, target_kd_);

    last_feedback_.position = position_feedback;
    last_feedback_.velocity = velocity_feedback;
    last_feedback_.torque = torque;
    last_feedback_.temperature = temperature;
  }

  void hold_position() {
    apply_target();
    publish_feedback();
  }

  void publish_feedback() {
    auto feedback = MotorFeedback();

    try {
      motor.receive_status_frame();
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "feedback read failed: %s", e.what());
    }

    auto [position_feedback, velocity_feedback, torque, temperature] =
        motor.return_data_pvtt();

    feedback.position = position_feedback;
    feedback.velocity = velocity_feedback;
    feedback.torque = torque;
    feedback.temperature = temperature;

    if (rclcpp::ok()) {
      feedback_pub_->publish(feedback);
    }
  }

private:
  rclcpp::Subscription<MotorMotionControl>::SharedPtr motion_sub_;
  rclcpp::Publisher<MotorFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  RobStrideMotor motor;

  double target_position_ = 0.0;
  double target_velocity_ = 0.0;
  double target_kp_ = 1.0;
  double target_kd_ = 0.1;
  double target_torque_ = 0.0;

  struct MotionFeedback {
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    double temperature = 0.0;
  } last_feedback_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<MotorControlSample>();

  rclcpp::spin(controller);

  rclcpp::shutdown();

  return 0;
}