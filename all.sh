#!/bin/bash
source /opt/ros/galactic/setup.bash
source install/setup.bash

# 启动complete_process.launch.py
gnome-terminal --title="Control Launch" -- bash -c "ros2 launch control complete_process_launch.py; exec bash"

# 启动arrow_detector节点
gnome-terminal --title="Arrow Detector" -- bash -c "ros2 run vision_pkg arrow_detector; exec bash"

# 启动linetrack节点
gnome-terminal --title="Line Tracking" -- bash -c "ros2 run vision_pkg linetrack; exec bash"

# 启动qr_detector节点
gnome-terminal --title="QR Detector" -- bash -c "ros2 run qr_detection qr_detector; exec bash"