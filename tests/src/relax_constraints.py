#!/usr/bin/env python3

import rospy
import moveit_commander
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
import sys

def set_max_path_constraints():
    # Initialize the moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('set_max_path_constraints_node', anonymous=True)

    # Create a MoveGroupCommander instance for the robot arm
    group_name = "manipulator_left"  # Replace with the name of your MoveGroup
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Get the names of all the links in the robot arm
    link_names = move_group.get_link_names()

    # Create a Constraints message
    constraints = Constraints()

    # Set maximum tolerances for each link
    for link_name in link_names:
        # Set maximum tolerances for PositionConstraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = link_name
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        position_constraint.constraint_region_shape.type = 1  # Type: SPHERE
        position_constraint.constraint_region_shape.dimensions.append(1.0)  # Radius
        position_constraint.weight = 1.0

        # Set maximum tolerances for OrientationConstraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = link_name
        orientation_constraint.orientation.x = 0.0
        orientation_constraint.orientation.y = 0.0
        orientation_constraint.orientation.z = 0.0
        orientation_constraint.orientation.w = 1.0
        orientation_constraint.absolute_x_axis_tolerance = 2 * 3.14159  # Maximum tolerance (360 degrees)
        orientation_constraint.absolute_y_axis_tolerance = 2 * 3.14159  # Maximum tolerance (360 degrees)
        orientation_constraint.absolute_z_axis_tolerance = 2 * 3.14159  # Maximum tolerance (360 degrees)
        orientation_constraint.weight = 1.0

        # Add the constraints to the Constraints message
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

    # Set the path constraints
    move_group.set_path_constraints(constraints)

    # Shutdown
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        set_max_path_constraints()
    except rospy.ROSInterruptException:
        pass
