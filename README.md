# Husky Robot

## Description
This codebase is used to interact with a dual-arm UR5e Husky A200 configuration at DiceLabs

## File Layout
- `docs/`: Contains project documentation, notes, and dependency configurations
- `imgs/`: Stores images related to the project
- `scripts/`: Holds automation scripts for codebase management
- `src/`: See robot source code here
- `tst/`: Unit test directory mirros source code for coverage

## Startup
If you want to turn on the robot, please refer to the physical startup procedure that has been documented [here](STARTUP.md)

From the software side of things, these scripts will have to be executed in this order

    setup_cb.sh
    kill_clearpath.sh
    start_brakes.sh
    start_brakes.sh
    start_driver.sh
    start_play.sh
    start_moveit.sh
    start_services.py
    start_arms.sh

You can now use the generic API laid out in [this file](src/integration/server_api/server.py) to call the desired systems

## Getting Started
Clone the codebase for access to source code scripts

    git clone https://github.com/DiceLabs/Husky

Since the codebase is primarily written in python, most robot functionality can be used without compilation. However, some code relies heavily on ROS, meaning their build system, catkin, will have to be used for "compiling" some parts of the code (just puts python files and config files in certain directories laid out by catkin). 
To do this, you can run this command in the root directory

    catkin_make

## Dependencies

    ROS Noetic, and in turn Ubuntu 20.04
    Realsense Cameras
    UR5e Client Library
    Dual Arm Husky Bringup

The robot itself will have to have the appropriate packages built and running correctly to be able to interact with it and to output endpoints from the ROS driver.