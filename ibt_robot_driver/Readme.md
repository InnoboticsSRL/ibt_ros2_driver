# IBT ROS2 Driver

- Launch with Python
```bash
python3 main.py --ros-args -r __ns:=/awtube
```

- Launch with ros2 launch
```bash
ros2 launch ibt_robot_driver ibt_robot_driver.launch.py
```

- Launch with ros2 run
```bash
ros2 run ibt_robot_driver ibt_robot_driver --ros-args -r __ns:=/awtube
```