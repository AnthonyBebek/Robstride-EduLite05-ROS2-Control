# RobStride ROS2 Motor Driver

This package is a ROS2 wrapper around the RobStride CAN motor interface. It was derived from the original RobStride sample code, which was written for the RobStride 02 actuator, but the same CAN control flow and ROS topic interface works correctly with the EDULite05 as well.

The node exposes a simple ROS interface for sending target motion commands and reading live motor feedback.

## What it does

The node subscribes to a motion command topic and publishes motor status on a feedback topic.

### Subscribed topic

- Topic: `/motion_control`
- Message type: `carmy_motor_controller/msg/MotorMotionControl`
- Fields:
  - `position` (`float64`): target position in radians
  - `velocity` (`float64`): desired feedforward velocity
  - `kp` (`float64`): proportional gain for position correction
  - `kd` (`float64`): derivative gain for damping
  - `torque` (`float64`): feedforward torque command

This is the command input you publish to move the actuator to a target position and tune the controller behavior.

### Published topic

- Topic: `/motor_feedback`
- Message type: `carmy_motor_controller/msg/MotorFeedback`
- Fields:
  - `position` (`float64`): current measured position
  - `velocity` (`float64`): current measured velocity
  - `torque` (`float64`): current estimated/commanded torque
  - `temperature` (`float64`): motor temperature in degrees Celsius

The feedback topic is published at roughly 19 Hz in normal operation.

## Startup

The easiest way to start the node is with the provided script:

```bash
bash ./start.bash
```

This script does the following:

1. runs `SetupCan.bash` to bring up the CAN interface
2. starts the Docker Compose environment
3. enters the ROS container
4. sources the ROS2 environment and the package install setup
5. launches the node:

```bash
ros2 run carmy_motor_controller rs_motor_ros2_node
```

## Example usage
Assuming the motor id is 3, (possible id's are 1,2,3 & 4)
Publish a target position:

```bash
ros2 topic pub /motion_control carmy_motor_controller/msg/MotorMotionControl "{motor_id=3, position: 1.57, velocity: 0.0, kp: 10.0, kd: 1.0, torque: 0.0}" -1
```

Read the live feedback:

```bash
ros2 topic echo /motor_feedback
```

## Notes

- The original source sample was written for the RobStride 02 actuator.
- This package has been validated with the EDULite05 and works correctly for ROS2 control and feedback over CAN.
- The core ROS interface is position-based and is designed for sending setpoints and reading motor state over topics.

## Files of interest

- `start.bash`: launches the ROS environment and starts the node
- `carmy_motor_controller/src/main.cpp`: ROS node callback and control loop
- `carmy_motor_controller/src/motor_cfg.cpp`: CAN communication and motor control logic
- `carmy_motor_controller/msg/MotorMotionControl.msg`: command message definition
- `carmy_motor_controller/msg/MotorFeedback.msg`: feedback message definition


