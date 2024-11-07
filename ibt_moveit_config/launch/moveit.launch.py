from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from typing import Any, List
from launch.substitutions import Command, FindExecutable


# ----------- Functions needed to work with file and or yaml -----------
def load_file(package_name: str, file_path: str) -> bytes | None:
    """
    Load and return the given file
    :param package_name: the ROS2 package in where the file is located
    :param file_path: the relative path of the file from the previously declared ROS2 package
    
    :return: The loaded file if it has been read properly, or None in case of error
    """
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name: str, file_path: str) -> Any | None:
    """
    Load and return the given YAML file
    :param package_name: The ROS2 package in where the file is located
    :param file_path: The relative path of the YAML file from the previously declared ROS2 package
    
    :return: The loaded file if it has been read properly, or None in case of error
    """
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def write_yaml(package_name: str, file_path: str, data: Any, use_hexadecimal: bool) -> None:
    """
    Write over an existing file
    :param package_name: The ROS2 package in where the file is located
    :param file_path: The relative path of the YAML file from the previously declared ROS2 package
    :param data: Data/object/element to be override to the original file
    :param use_hexadecimal: Flag to convert all integers number in hexadecimal
    
    :return: None
    """
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'w') as fp:
            # Create a custom dumper to represent integers as hexadecimal numbers if use_hexadecimal is True
            class MySafeDumper(yaml.SafeDumper):
                pass

            if use_hexadecimal:
                def hex_representer(dumper, data):
                    """
                    Convert integers number in hexadecimal

                    :param dumper: YAML dumper object provided by YAML library
                    :param data: The integer value to be converted to hexadecimal.

                    :return:  A YAML representation of the integer in hexadecimal format.
                    """
                    return dumper.represent_scalar('tag:yaml.org,2002:int', hex(data))

                MySafeDumper.add_representer(int, hex_representer)

            # Dump the data using the custom dumper
            yaml.dump(data, fp, Dumper=MySafeDumper, default_flow_style=False, sort_keys=False)
    except EnvironmentError:
        return None


def modify_yaml(package_name: str, file_path: str, keys: List[str | int], new_value: Any,
                use_hexadecimal: bool = False) -> None:
    """
    Modify a value in a YAML file at a specified path using the given keys.

    This function loads a YAML file, traverses its structure using the provided keys, and updates the value at the specified key
    with the new_value. The modified data is then written back to the YAML file. Optionally, you can choose to represent integer values
    in hexadecimal format when writing the data back.

    :param package_name: The ROS2 package in where the file is located
    :param file_path: The relative path of the YAML file from the previously declared ROS2 package
    :param keys: A list of keys (strings or integers) specifying the traversal path within the YAML structure.
    :param new_value: The new value to replace the existing value at the specified key.
    :param use_hexadecimal: A boolean flag indicating whether to represent integer values in hexadecimal format (default is False).

    :return: None
    """
    try:
        data = load_yaml(package_name, file_path)
        current_data = data

        # Traverse the YAML structure using the provided keys
        for key in keys[:-1]:
            if isinstance(current_data, list):
                key = int(key)
            current_data = current_data[key]

        # Set the new value at the specified key
        last_key = keys[-1]
        if isinstance(current_data, list):
            last_key = int(last_key)
        current_data[last_key] = new_value

        # Write the modified data back to the YAML file
        write_yaml(package_name, file_path, data, use_hexadecimal)
    except EnvironmentError:
        return None


def generate_launch_description():
    # ------ Launching parameters ------
    gbc_url = LaunchConfiguration('gbc_url', default='ws://localhost:9001/ws')
    use_fake = LaunchConfiguration('use_fake', default=True)

    # -------- Pilz motion planner --------
    start_state_max_bounds_error = 0.1
    allowed_execution_duration_scaling = 1.2
    allowed_goal_duration_margin = 0.5
    allowed_start_tolerance = 0.01
    publish_planning_scene = True
    publish_geometry_updates = True
    publish_state_updates = True
    publish_transforms_updates = True
    publish_robot_description = True
    publish_robot_description_semantic = True
    publish_robot_description_kinematics = True
    moveit_package = 'ibt_moveit_config'
    robot_name = 'awtube'

    # ---------- Robot description ------
    xacro_file = os.path.join(get_package_share_directory(moveit_package), 'config/' + robot_name + '.urdf.xacro')
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])
    robot_description_diz = {'robot_description': robot_description}

    # ---------- Rviz ----------
    rviz_config = os.path.join(get_package_share_directory(moveit_package), 'config', 'config.rviz')

    # ---------- Pilz Planning -------------
    planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
            'capabilities': 'pilz_industrial_motion_planner/MoveGroupSequenceAction '
                            'pilz_industrial_motion_planner/MoveGroupSequenceService',
            'start_state_max_bounds_error': start_state_max_bounds_error
        }
    }

    # ---------- Trajectory Execution -------------
    trajectory_execution_config = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': allowed_execution_duration_scaling,
        'trajectory_execution.allowed_goal_duration_margin': allowed_goal_duration_margin,
        'trajectory_execution.allowed_start_tolerance': allowed_start_tolerance,
        'trajectory_execution.execution_duration_monitoring': False,
    }

    # ---------- Planning Scene Monitor -------------
    planning_scene_monitor_parameters = {
        'publish_planning_scene': publish_planning_scene,
        'publish_geometry_updates': publish_geometry_updates,
        'publish_state_updates': publish_state_updates,
        'publish_transforms_updates': publish_transforms_updates,
        "publish_robot_description": publish_robot_description,
        "publish_robot_description_semantic": publish_robot_description_semantic,
        "publish_robot_description_kinematics": publish_robot_description_kinematics,
    }

    # ---------- Robot params -------------
    robot_description_semantic = {
        'robot_description_semantic': load_file(moveit_package, 'config/' + robot_name + '.srdf')
    }
    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(moveit_package, 'config/' + 'kinematics.yaml')
    }
    robot_description_planning = {
        'robot_description_planning': load_yaml(moveit_package, 'config/' + 'joint_limits.yaml')
    }

    # ---------- MoveIt Controllers -------------
    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml(moveit_package, 'config/' + 'moveit_controllers.yaml'),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }

    # Nodes
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_config,
            trajectory_execution_config,
            moveit_controllers,
            planning_scene_monitor_parameters
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': False},
            robot_description_diz]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['/awtube/joint_states']}],
    )

    ibt_robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ibt_robot_driver'), 'launch', 'ibt_robot_driver.launch.py')
        ),
        launch_arguments={
            'gbc_url': gbc_url,
            'use_fake': use_fake
        }.items()
    )

    # nodes to be launched
    nodes = [
        ibt_robot_driver,
        joint_state_publisher,
        robot_state_publisher,
        run_move_group_node,
        rviz_node
    ]

    return LaunchDescription(nodes)
