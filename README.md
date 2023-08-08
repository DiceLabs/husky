# Start-up Guide

## Establishing wireless connection in lab
Connect to **SDSU_Dice_Labs**

## Potential network connection problems & fixes (optional)
The computer or network equipment may be having trouble connecting hostnames to the corresponding ips
```
sudo nano /etc/hosts
```
Below the line that contatins 127.0.1.1, add this line
```
146.244.98.51   cpr-a200-0876
```
It may be required that you add the connection of ip to hostname of your own device into the robot. If that is the case then you will need to get onto a device that already has an established connection to the robot, SSH into the robot and add your configuration into the robots hosts. This is done below while in a terminal that is with the robots os.
```
sudo nano /etc/hosts
```
Below the line that contatins 127.0.1.1 and other devices that are used in the lab, add a line contataining
```
<your_devices_ip_while_on_SDSU_Dice_Labs_wifi>   <your_devices_hostname>
```




## Creating workspace environment
Remember this is for working in Ubuntu 20.04

These steps can be found on [Ros Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)

Open a terminal


###### Setup your sources.list
Setup your computer to accept software from packages.ros.org.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

###### Set up your keys
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
###### Installation
```
sudo apt update
```
```
sudo apt install ros-noetic-desktop-full
```
###### For installing additional packages (format)
```
sudo apt install ros-noetic-PACKAGE
```

###### Ensure installation was successful
```
apt search ros-noetic
```

###### Environment setup
**You must source this script in every bash terminal you use ROS in**
```
source /opt/ros/noetic/setup.bash
```
**If you want every terminal defaulted to sourcing this Ros version**
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
###### Dependencies for building packages
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
###### Initialize rosdep
```
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
## Creating Ros Workspace
Create workspace directories
```
cd
mkdir husky_ws && cd husky_ws
mkdir src && cd src
```
**You need to be on the same wifi network as the robot**
```
scp -r administrator@cpr-a200-0876:~/catkin_ws/src/* .
```
If you run into an error where cpr-a200-0876 can't be found follow the instructions in [network problems section](https://github.com/DiceLabs/Husky/blob/developed_ROS_packages/README.md#potential-network-connection-problems--fixes-optional) then try again

Install dependencies for these packages that have just been transferred over to device

```
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -r -y
catkin_make
source ~/husky_ws/devel/setup.bash
```


## Useful Bash Aliases/Hotkey(commands)
When the process of running common terminal commands gets too repetitive it may be much easier to simplify an entire command to one word or phrase that you can remember. This can be done in the .bashrc script with the following two methods:

**Mehtod 1:**

```
echo "alias husky='ssh administrator@146.244.98.51'" >> ~/.bashrc
source ~/.bashrc
```
**Method 2:**
```
sudo nano ~/.bashrc
```
Then add aliases in the script with this format
```
alias <word_or_phrase_you_want_hotkey_o_be>='<command>'
```
**Example:**
```
alias husky='ssh administrator@146.244.98.51'
```

Save the file and close out of editting window then type this into terminal
```
source ~/.bashrc
```
This will apply all changes made into bashrc

### Useful aliases
```
alias husky='ssh administrator@146.244.98.51'
alias husky_export='export ROS_MASTER_URI=http://cpr-a200-0876:11311'
alias husky_src='source ~/catkin_ws/devel/setup.bash'
```














# Object Perception and navigation
The `ml_test` package enables object perception functionality, as well as navigation to perceived objects. As of now, the navigation script cannot handle dynamic objects, or objects which are not part of the originally generated map. A modified script will be made to try to fix this issue.


# Robot arm manipulation
The `test` package enables robot arm end-effector-to-perceived object interaction. The script in this package also handles final orientation of the end effector so it correctly interacts with object. This package has recently been tested with "bottle" objects.
