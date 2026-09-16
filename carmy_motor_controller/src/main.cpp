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

class MotorController : public rclcpp::Node {
public:
  MotorController(std::vector<uint8_t>& motor_ids, std::vector<int>& motor_types)
  // Initialize the ROS2 node with the name "motor_control_set_node" and create a RobStrideMotor instance
      : rclcpp::Node("motor_controller_node") {

        // Create RobStrideMotor instances for each motor ID and type
        for (size_t i = 0; i < motor_ids.size(); ++i) {
          motors_.try_emplace(motor_ids[i], "can0", 0xFF, motor_ids[i], motor_types[i]);
        }

        for (auto& [id, motor] : motors_) {
          motor.Get_RobStrite_Motor_parameter(0x7005);
          usleep(1000);
          motor.enable_motor();
          usleep(1000);
        }
    // Create a subscription to the "motion_control" topic and a publisher for the "motor_feedback" topic
    motion_sub_ = this->create_subscription<MotorMotionControl>(
        "motion_control", 10,
        std::bind(&MotorController::motion_callback, this,
                  std::placeholders::_1));

    feedback_pub_ = this->create_publisher<MotorFeedback>("motor_feedback", 10);

    // Create a timer to call the hold_position function every 20 milliseconds
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MotorController::hold_position, this));
  }

  // Destructor to disable all motors when the node is shut down
  ~MotorController() {
    for (auto& [id, motor] : motors_) {
      motor.Disenable_Motor(0);
    }
  }

  void motion_callback(const MotorMotionControl::SharedPtr msg) {
    // Check if the motor ID from the message exists in the motors_ map
    auto it = motors_.find(msg->motor_id);
    if (it == motors_.end()) {
      RCLCPP_WARN(this->get_logger(), "Motor ID %d not found", msg->motor_id);
      return;
    }

    // Update the target position, velocity, and connection status for the specified motor ID
    target_position_[msg->motor_id] = msg->position;
    target_velocity_[msg->motor_id] = msg->velocity;
    target_connected_[msg->motor_id] = true;
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
    // Check if the motor ID exists in the motors_ map
    auto it = motors_.find(motor_id);
    if (it == motors_.end()) return;

    // Send the motion command to the motor and receive feedback
    auto [position_feedback, velocity_feedback, torque, temperature] =
        it->second.send_motion_command(target_torque_[motor_id], target_position_[motor_id],
                                       target_velocity_[motor_id], target_kp_[motor_id], target_kd_[motor_id]);
    
    last_feedback_[motor_id].position = position_feedback;
    last_feedback_[motor_id].velocity = velocity_feedback;
    last_feedback_[motor_id].torque = torque;
    last_feedback_[motor_id].temperature = temperature;
  }

  void hold_position() {
    // Apply the last known target position for each motor to maintain its position
    for (auto& [motor_id, motor] : motors_) {
      apply_target(motor_id);
    }
    publish_feedback();
  }

  void publish_feedback() {
    // Publish feedback for each motor
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

      // If the motor is not connected, change the connection status, and log a warning message
      if (position_feedback == 0.0 && velocity_feedback == 0.0 && torque == 0.0 && temperature == 0.0) {
        // Motors with null feedback are considered disconnected
        RCLCPP_WARN(this->get_logger(), "Motor %d is not detected!", motor_id);
        feedback.connected = false;
      } else {
        feedback.connected = true;
      }

      // Update the last feedback for the motor
      feedback.position = position_feedback;
      feedback.velocity = velocity_feedback;
      feedback.torque = torque;
      feedback.temperature = temperature;

      // Publish the feedback message if the ROS2 node is still running
      if (rclcpp::ok()) {
        feedback_pub_->publish(feedback);
      }
    }
  }

private:
  // ROS2 subscription and publisher for motor control and feedback
  rclcpp::Subscription<MotorMotionControl>::SharedPtr motion_sub_;
  rclcpp::Publisher<MotorFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::map<uint8_t, RobStrideMotor> motors_;

  std::map<uint8_t, double> target_position_;
  std::map<uint8_t, double> target_velocity_;
  std::map<uint8_t, double> target_kp_;
  std::map<uint8_t, double> target_kd_;
  std::map<uint8_t, double> target_torque_;
  std::map<uint8_t, bool> target_connected_;

  // Structure to hold the last feedback received from each motor
  struct MotionFeedback {
    bool connected = false;
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    double temperature = 0.0;
  };
  std::map<uint8_t, MotionFeedback> last_feedback_;
};

int main(int argc, char **argv) {
  // Defining the motor IDs to be used in the node
  std::vector<uint8_t> motor_ids = {0x01, 0x02, 0x03, 0x04};
  std::vector<int> motor_types = {5, 5, 5, 5}; // All motors are EDULite05s

  rclcpp::init(argc, argv);

  auto controller = std::make_shared<MotorController>(motor_ids, motor_types);
  

  rclcpp::spin(controller);

  rclcpp::shutdown();

  return 0;
}