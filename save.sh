#!/bin/bash

# 设置保存地图的目标目录
MAP_DIR="/home/davinci-mini/CHEN2/src/racecar/maps"
MAP_NAME="ai_map"



# 运行 map_saver_cli 命令保存地图
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME"