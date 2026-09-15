#!/bin/bash

# --- 1. Safely Initialize Virtual CAN (vcan0) ---
sudo modprobe vcan
# Only add vcan0 if it doesn't already exist
if ! ip link show vcan0 > /dev/null 2>&1; then
    sudo ip link add dev vcan0 type vcan
fi
sudo ip link set vcan0 up

# --- 2. Clean Up Existing Interface and Daemon ---
if ip link show can0 > /dev/null 2>&1; then
    sudo ip link set can0 down 2>/dev/null
fi
sudo killall slcand 2>/dev/null

# --- 3. Dynamically Find the USB Adapter Node ---
# This checks if /dev/ttyACM0 or /dev/ttyACM1 exists
TTY_DEV=$(ls /dev/ttyACM* 2>/dev/null | head -n 1)

if [ -z "$TTY_DEV" ]; then
    echo "Error: No CAN adapter found on /dev/ttyACM*. Is it plugged in?"
    exit 1
fi

echo "Found adapter on $TTY_DEV. Initialising..."

# --- 4. Load Driver Module and Bind slcand at 1 Mbps (-s8) ---
sudo modprobe slcan
sudo slcand -o -c -s8 "$TTY_DEV" can0

# Give the kernel half a second to create the interface string
sleep 0.5

# --- 5. Turn Network Link Online ---
if ip link show can0 > /dev/null 2>&1; then
    sudo ip link set can0 up txqueuelen 100
    echo "Native SocketCAN interface (can0) is UP at 1 Mbps!"
else
    echo "Error: Failed to create can0 interface."
    exit 1
fi
