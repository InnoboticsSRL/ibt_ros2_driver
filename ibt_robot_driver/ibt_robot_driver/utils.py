def print_trajectory_goal(goal):
    print("FollowJointTrajectory_Goal:")
    print("  Trajectory:")
    print("    Header:")
    print(f"      Stamp: sec={goal.trajectory.header.stamp.sec}, nanosec={goal.trajectory.header.stamp.nanosec}")
    print(f"      Frame ID: {goal.trajectory.header.frame_id}")

    print("    Joint Names:", ", ".join(goal.trajectory.joint_names))

    print("    Points:")
    for i, point in enumerate(goal.trajectory.points):
        print(f"      Point {i + 1}:")
        print(f"        Positions: {point.positions}")
        print(f"        Velocities: {point.velocities}")
        print(f"        Accelerations: {point.accelerations}")
        print(f"        Effort: {point.effort}")
        print(f"        Time from Start: sec={point.time_from_start.sec}, nanosec={point.time_from_start.nanosec}")

    print("  Multi DOF Trajectory:")
    print(
        f"    Header Stamp: sec={goal.multi_dof_trajectory.header.stamp.sec}, nanosec={goal.multi_dof_trajectory.header.stamp.nanosec}")
    print(f"    Frame ID: {goal.multi_dof_trajectory.header.frame_id}")
    print("    Joint Names:", ", ".join(goal.multi_dof_trajectory.joint_names))
    print("    Points:", goal.multi_dof_trajectory.points)

    print("  Path Tolerance:", goal.path_tolerance)
    print("  Component Path Tolerance:", goal.component_path_tolerance)
    print("  Goal Tolerance:", goal.goal_tolerance)
    print("  Component Goal Tolerance:", goal.component_goal_tolerance)
    print("  Goal Time Tolerance:")
    print(f"    sec={goal.goal_time_tolerance.sec}, nanosec={goal.goal_time_tolerance.nanosec}")


def bool_list_to_uint8(bool_list):
    if len(bool_list) != 8:
        raise ValueError("The list must contain exactly 8 boolean values.")
    return sum((1 if bit else 0) << (7 - i) for i, bit in enumerate(bool_list))

def uint8_to_bool_list(uint8_value):
    if not (0 <= uint8_value <= 255):
        raise ValueError("The value must be a uint8 (between 0 and 255).")
    return [(uint8_value & (1 << (7 - i))) != 0 for i in range(8)]