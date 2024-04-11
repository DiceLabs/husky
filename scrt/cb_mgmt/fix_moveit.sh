#!/bin/bash

temp_launch_moveit()
{
    timeout 3s roslaunch sds04_husky_moveit_config husky_dual_ur_robotiq_2f_85_moveit_planning_execution.launch
}

fix_moveit_config()
{
    MOVEIT_TARGET_DIR="$HOME/catkin_ws/src/sds04_husky/sds04_husky_moveit_config/launch/husky_dual_ur_robotiq_2f_85_moveit_planning_execution.launch"
    cp $HOME/husky/docs/no_namespace_dual_launch.xml $MOVEIT_TARGET_DIR
    temp_launch_moveit
    cp $HOME/husky/docs/namespaced_dual_launch.xml $MOVEIT_TARGET_DIR
}

fix_moveit_config