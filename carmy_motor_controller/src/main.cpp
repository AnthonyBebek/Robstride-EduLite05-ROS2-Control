#include "motor_ros2/motor_cfg.h"
#include "carmy_motor_controller/msg/motor_motion_control.hpp"
#include "carmy_motor_controller/msg/motor_feedback.hpp"
#include "stdint.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
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
  MotorControlSample(std::vector<uint8_t>& motor_ids)
  // Initialize the ROS2 node with the name "motor_control_set_node" and create a RobStrideMotor instance
      : rclcpp::Node("multi_motor_control_node") {

        for (uint8_t id : motor_ids) {
          motors_.try_emplace(id, "can0", 0xFF, id, 5);
        }

        for (auto& [id, motor] : motors_) {
          motor.Get_RobStrite_Motor_parameter(0x7005);
          usleep(1000);
          motor.enable_motor();
          usleep(1000);
        }

    motion_sub_ = this->create_subscription<MotorMotionControl>(
        "motion_control", 10,
        std::bind(&MotorControlSample::motion_callback, this,
                  std::placeholders::_1));

    feedback_pub_ = this->create_publisher<MotorFeedback>("motor_feedback", 10);

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MotorControlSample::hold_position, this));
  }

  ~MotorControlSample() {
    for (auto& [id, motor] : motors_) {
      motor.Disenable_Motor(0);
    }
  }

  void motion_callback(const MotorMotionControl::SharedPtr msg) {
    auto it = motors_.find(msg->motor_id);
    if (it == motors_.end()) {
      RCLCPP_WARN(this->get_logger(), "Motor ID %d not found", msg->motor_id);
      return;
    }

    target_position_[msg->motor_id] = msg->position;
    target_velocity_[msg->motor_id] = msg->velocity;
    target_kp_[msg->motor_id] = msg->kp;
    target_kd_[msg->motor_id] = msg->kd;
    target_torque_[msg->motor_id] = msg->torque;

    apply_target(msg->motor_id);

    RCLCPP_INFO(this->get_logger(),
                "motion cmd (motor %d): pos=%.3f vel=%.3f kp=%.3f kd=%.3f torque=%.3f",
                msg->motor_id, msg->position, msg->velocity, msg->kp, msg->kd,
                msg->torque);
  }

  void apply_target(uint8_t motor_id) {
    auto it = motors_.find(motor_id);
    if (it == motors_.end()) return;

    auto [position_feedback, velocity_feedback, torque, temperature] =
        it->second.send_motion_command(target_torque_[motor_id], target_position_[motor_id],
                                       target_velocity_[motor_id], target_kp_[motor_id], target_kd_[motor_id]);

    last_feedback_[motor_id].position = position_feedback;
    last_feedback_[motor_id].velocity = velocity_feedback;
    last_feedback_[motor_id].torque = torque;
    last_feedback_[motor_id].temperature = temperature;
  }

  void hold_position() {
    for (auto& [motor_id, motor] : motors_) {
      apply_target(motor_id);
    }
    publish_feedback();
  }

  void publish_feedback() {
    for (auto& [motor_id, motor] : motors_) {
      auto feedback = MotorFeedback();
      feedback.motor_id = motor_id;

      try {
        motor.receive_status_frame();
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "feedback read failed for motor %d: %s", motor_id, e.what());
        continue;
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
  }

private:
  rclcpp::Subscription<MotorMotionControl>::SharedPtr motion_sub_;
  rclcpp::Publisher<MotorFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::map<uint8_t, RobStrideMotor> motors_;

  std::map<uint8_t, double> target_position_;
  std::map<uint8_t, double> target_velocity_;
  std::map<uint8_t, double> target_kp_;
  std::map<uint8_t, double> target_kd_;
  std::map<uint8_t, double> target_torque_;

  struct MotionFeedback {
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    double temperature = 0.0;
  };
  std::map<uint8_t, MotionFeedback> last_feedback_;
};

int main(int argc, char **argv) {
  std::vector<uint8_t> motor_ids = {0x01, 0x02, 0x03, 0x04};

  rclcpp::init(argc, argv);

  auto controller = std::make_shared<MotorControlSample>(motor_ids);
  

  rclcpp::spin(controller);

  rclcpp::shutdown();

  return 0;
}