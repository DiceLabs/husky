#!/bin/bash

start_moveit()
{
    roslaunch sds04_husky_moveit_config husky_dual_ur_robotiq_2f_85_moveit_planning_execution.launch
}

source fix_moveit.sh

start_moveit