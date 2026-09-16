# RobStride ROS2 Motor Driver

This package is a ROS2 wrapper around the RobStride CAN motor interface. It was derived from the original RobStride sample code, which was written for the RobStride 02 actuator, but the same CAN control flow and ROS topic interface works correctly with the EDULite05 as well.

The node exposes a simple ROS interface for sending target motion commands and reading live motor feedback.

## What it does

The node subscribes to a motion command topic and publishes motor status on a feedback topic.

> [!IMPORTANT]  
> As this is built for the carmy continuum arm project, which uses 4 RobStride 04 actuators, where the motors have preconfigured to use 
motor_ids `0x01`, `0x02`, `0x03` and `0x04`. 

Motor types and addresses can be reconfigured in the `main.cpp` file under the main function.

### Subscribed topic

- Topic: `/motion_control`
- Message type: `carmy_motor_controller/msg/MotorMotionControl`
- Fields:
  - `motor_id` (`uint8`): ID of the desired motor
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
  - `motor_id` (`uint8`): ID of the motor
  - `connected` (`bool`): motor connection status
  - `position` (`float64`): current measured position
  - `velocity` (`float64`): current measured velocity
  - `torque` (`float64`): current estimated/commanded torque
  - `temperature` (`float64`): motor temperature in degrees Celsius

>[!NOTE] 
> The motors encoder percision is tied to the current motor position, meaning the further the motor is from it's home percision, the less percise the motor is in it's reported percision. Therefore to ensure a 16 bit percision, motors are clamped to `18.20π to -18.20π`

The feedback topic is published at roughly 19 Hz in normal operation.

## Startup

>  [!NOTE]
> As this repo tried to automatically configure the CAN adapter on startup, it may not work as expected on different CAN bus adapters. Hence, you will need to manually update the `SetupCan.bash` script to automatically setup the `can0` interface. This repo is setup for a generic unbranded adapter found on [amazon](https://www.amazon.com.au/Module-Converter-Analyzer-Adapter-Support/dp/B0DRVCN5CD?crid=1XT1EGEQRS44M&dib=eyJ2IjoiMSJ9.OChvO9LpMDYewJISydx1p3rCIxp6Xxjk1lzS6OVGzxZhDL4cnVyDfziI5QR1UGEfooIhTexzh9UtWTINnop70cYmYFN8efGINeyI_m8w-gwUqklO8GqWoOys3ro2_I0NCpYIoskCfxj1urrptqajYIsNLh-Ltj8VL_6fmzSV9Ex0mkMctkq3viCK_FtjUExhdDN82-CWHg_uJ7r7z3dESKIXqbe1VlwY3cuQjKdem-GJg19roODn6gvv8JAjF_k1uuIH_atvX1YeEtfiqHD-3Vb3z9D-8yAsOlUwUJO1XO4.me7AzevBUkkMmzN0c7chpE_somlwVWVvmIzJ6XncAxI&dib_tag=se&keywords=can+adapter&qid=1789293031&sprefix=can+adapt%2Caps%2C237&sr=8-9)

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
Assuming the motor id is `3`, (possible id's are `1`,`2`,`3` & `4`)
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
- This package is preconfigured to work wiith the Robstride 04 actuators, and later planned to be fully tested with these actuators in the near future.
- The core ROS interface is position-based and is designed for sending setpoints and reading motor state over topics.

## Files of interest

- `start.bash`: launches the ROS environment and starts the node
- `carmy_motor_controller/src/main.cpp`: ROS node callback and control loop
- `carmy_motor_controller/src/motor_cfg.cpp`: CAN communication and motor control logic
- `carmy_motor_controller/msg/MotorMotionControl.msg`: command message definition
- `carmy_motor_controller/msg/MotorFeedback.msg`: feedback message definition


