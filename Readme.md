# IBT ROS2 Driver

## Getting Started
1. Clone IBT descriptions
```bash
git clone https://github.com/InnoboticsSRL/ibt_ros2_description.git
```
2. Compile the entire workspace
```bash
colcon build --symlink-install
```
3. Launch the driver with
```bash
ros2 launch ibt_moveit_config moveit.launch.py
```
or Launch only ibt_robot_driver
```bash
ros2 launch ibt_robot_driver ibt_robot_driver.launch.py
```