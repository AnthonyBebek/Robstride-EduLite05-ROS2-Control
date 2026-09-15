#!/bin/bash

# First setup the CAN interface
bash ./SetupCan.bash

# This should start the ROS2 enviroment, then run the motor controller node
CONTAINER_NAME="carmy_ros"

SOURCE_COMMAND="source /opt/ros/humble/setup.bash && source /ws/carmy_motor_controller/install/setup.bash"

START_NODE_COMMAND="$SOURCE_COMMAND && ros2 run carmy_motor_controller rs_motor_ros2_node"

echo "Starting ROS2 environment..."
sudo docker compose up -d
docker exec -it $CONTAINER_NAME /bin/bash -c "$START_NODE_COMMAND"