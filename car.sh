#!/bin/bash
# Function to handle termination
terminate() {
  echo "终止所有后台进程..."
  kill $Run_car
  wait $Run_car
  echo "所有进程已终止。"
}

# Trap Ctrl+C (SIGINT)
trap terminate SIGINT

# 启动 ROS 2 launch 文件


ros2 launch racecar Run_car.launch.py &
Run_car=$!
wait $Run_car