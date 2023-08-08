# Start-up Guide
## Creating workspace environment
Remember this is for working in Ubuntu 20.04

These steps can be found on [Ros Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
Open a terminal


### Setup your sources.list
Setup your computer to accept software from packages.ros.org.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### Set up your keys
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
### Installation
```
sudo apt update
```
```
sudo apt install ros-noetic-desktop-full
```
### For installing additional packages
```
sudo apt install ros-noetic-PACKAGE
```

### Ensure installation was successful
```
apt search ros-noetic
```

### Environment setup
##### You must source this script in every bash terminal you use ROS in.
```
source /opt/ros/noetic/setup.bash
```
##### If you want every terminal defaulted to sourcing this Ros version
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### Dependencies for building packages
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
### Initialize rosdep
```
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
## Creating Ros Workspace
```
cd
mkdir husky_ws && cd husky_ws
mkdir src && cd src
```


















# Object Perception and navigation
The `ml_test` package enables object perception functionality, as well as navigation to perceived objects. As of now, the navigation script cannot handle dynamic objects, or objects which are not part of the originally generated map. A modified script will be made to try to fix this issue.


# Robot arm manipulation
The `test` package enables robot arm end-effector-to-perceived object interaction. The script in this package also handles final orientation of the end effector so it correctly interacts with object. This package has recently been tested with "bottle" objects.
