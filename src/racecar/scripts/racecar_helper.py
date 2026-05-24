#!/usr/bin/env python3

import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

WORKSPACE = Path(os.environ.get('RACECAR_WS', '/home/davinci-mini/CHEN2'))
MAP_FILE = WORKSPACE / 'src/racecar/maps/race_map'
MAP_YAML = WORKSPACE / 'src/racecar/maps/race_map.yaml'
LOG_DIR = WORKSPACE / 'logs'

processes = {}


def env_command(command):
    return f'cd {WORKSPACE} && source install/setup.bash && {command}'


def start_process(name, command, log_output=False):
    if name in processes and processes[name].poll() is None:
        print(f'{name} 已经在运行')
        return

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_file = None
    stdout = None
    stderr = None

    if log_output:
        log_path = LOG_DIR / f'{name}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
        log_file = open(log_path, 'w')
        stdout = log_file
        stderr = subprocess.STDOUT
        print(f'{name} 输出已写入日志: {log_path}')

    proc = subprocess.Popen(
        ['bash', '-lc', env_command(command)],
        stdin=sys.stdin,
        stdout=stdout,
        stderr=stderr,
        preexec_fn=os.setsid,
    )
    proc.log_file = log_file
    processes[name] = proc
    print(f'{name} 已启动')


def run_once(name, command):
    print(f'执行: {name}')
    subprocess.run(['bash', '-lc', env_command(command)])


def run_foreground(name, command):
    print(f'进入{name}，按 Ctrl+C 返回菜单')
    subprocess.run(['bash', '-lc', env_command(command)])


def stop_process(name):
    proc = processes.get(name)
    if not proc or proc.poll() is not None:
        print(f'{name} 没有在运行')
        return

    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5)

    if getattr(proc, 'log_file', None):
        proc.log_file.close()
    print(f'{name} 已停止')


def stop_all():
    for name in list(processes):
        stop_process(name)


def show_menu():
    print('\n========== racecar 一键脚本 ==========')
    print('1. 启动小车底盘/雷达/IMU/EKF（刷屏输出写日志）')
    print('2. 启动 GMapping 建图')
    print('3. 启动键盘控制')
    print('4. 保存地图')
    print('5. 启动正式比赛导航')
    print('6. 打开 RViz')
    print('7. 选择导航点模式')
    print('8. 选择停车点模式')
    print('9. 确认点位')
    print('10. ARM 比赛')
    print('11. 清空点位')
    print('12. 取消比赛并停车')
    print('13. 打开雷达')
    print('14. 关闭雷达')
    print('15. 停止建图和键盘控制')
    print('16. 停止全部由本脚本启动的进程')
    print('0. 退出')


def main():
    try:
        while True:
            show_menu()
            choice = input('请输入编号: ').strip()

            if choice == '1':
                start_process('run_car', 'ros2 launch racecar Run_car.launch.py', log_output=True)
            elif choice == '2':
                start_process('gmapping', 'ros2 launch slam_gmapping slam_gmapping.launch.py')
            elif choice == '3':
                run_foreground('键盘控制', 'ros2 run racecar racecar_teleop.py')
            elif choice == '4':
                run_once('保存地图', f'ros2 run nav2_map_server map_saver_cli -f {MAP_FILE}')
            elif choice == '5':
                start_process('run_race', f'ros2 launch racecar Run_race.launch.py map:={MAP_YAML}')
            elif choice == '6':
                start_process('rviz', 'rviz2')
            elif choice == '7':
                run_once('选择导航点模式', 'ros2 service call /race/select_waypoints std_srvs/srv/Trigger {}')
            elif choice == '8':
                run_once('选择停车点模式', 'ros2 service call /race/select_parking std_srvs/srv/Trigger {}')
            elif choice == '9':
                run_once('确认点位', 'ros2 service call /race/confirm_points std_srvs/srv/Trigger {}')
            elif choice == '10':
                run_once('ARM 比赛', 'ros2 service call /race/arm std_srvs/srv/Trigger {}')
            elif choice == '11':
                run_once('清空点位', 'ros2 service call /race/clear_points std_srvs/srv/Trigger {}')
            elif choice == '12':
                run_once('取消比赛并停车', 'ros2 service call /race/cancel std_srvs/srv/Trigger {}')
            elif choice == '13':
                run_once('打开雷达', 'ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 "{data: 1}"')
            elif choice == '14':
                run_once('关闭雷达', 'ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 "{data: 0}"')
            elif choice == '15':
                stop_process('gmapping')
                stop_process('teleop')
            elif choice == '16':
                stop_all()
            elif choice == '0':
                stop_all()
                return
            else:
                print('编号无效')

            time.sleep(0.3)
    except KeyboardInterrupt:
        print('\n正在退出...')
        stop_all()


if __name__ == '__main__':
    main()
