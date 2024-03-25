#!/bin/bash

setup_environment()
{
    cd $HOME/husky
    catkin_make
    source source.sh
    bash  install.sh
    bash  build_urclient.sh
}

temp_launch_moveit()
{
    bash roslaunch sds04_husky_moveit_config husky_dual_ur_robotiq_2f_85_moveit_planning_execution.launch &
    ROSLAUNCH_PID=$!
    sleep 1
    kill $ROSLAUNCH_PID
}

fix_moveit_config()
{
    MOVEIT_TARGET_DIR="$HOME/catkin_ws/src"
    cp docs/namespace_dual_launch.xml $MOVEIT_TARGET_DIR
    temp_launch_moveit
    cp docs/no_namespace_dual_launch.xml $MOVEIT_TARGET_DIR
}

start_driver()
{
    bash roslaunch husky_ur_bringup husky_dual_ur_bringup.launch &
}

start_moveit()
{
    
    bash roslaunch sds04_husky_moveit_config husky_dual_ur_robotiq_2f_85_moveit_planning_execution.launch &
}

start_arm_servers()
{
    bash start_arms.sh
}

release_brakes()
{
    ur_interface.py --action r
}

press_play()
{
    ur_interface.py --action p
}

setup_cb()
{
    setup_environment
    # start_driver
    # release_brakes
    # fix_moveit_config
    # start_moveit
    # press_play
    # start_arm_servers
}

main()
{
    if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
        setup_cb "$@"
    fi
}

main