# Start-up Guide

## E-STOP
There are two main methods the user can E-stop the robot. **Pressing the red button on the back of the robot** or **pressing the top right black button on the controller**.

## Establishing wireless connection in lab
Connect to **SDSU_Dice_Labs**


## Installing ROS
**Acknowledge** this is for working in **Ubuntu 20.04** environment

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


### Useful Bash Aliases/Hotkey(commands)
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

#### Useful aliases
```
alias husky='ssh administrator@146.244.98.51'
alias husky_export='export ROS_MASTER_URI=http://cpr-a200-0876:11311'
alias husky_src='source ~/catkin_ws/devel/setup.bash'
```


### Potential network connection problems & fixes (optional)
The computer or network equipment may be having trouble connecting hostnames to the corresponding ip's

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

## UR arms
###### UNDERSTAND
The robots UI monitor on the back can only operate one arm at a time
##### Power on arms
On the robot there are two buttons under the robots UI on the right side of the panel labeled left and right. These are the power buttons. Press and release one of the buttons, then wait until you hear a fan start up, after the fan has started up, you will press and release the other button and it too will have a fan startup. It does not matter what order you press the buttons in. The bootup process will take approx. 2 minutes.
<PICTURES>

##### Working with UI and controller to activate the arms
Once the arms have started up the UI on the back of the robot will be on and most likely have a Robot Emergency Stop warning. This is simply stating the last time the robot was in use it was E-stopped. You can just ignore this and click **Not now**

Then you will click the red button on the bottom left side of the UI 
<picture_UI>

The UI should now look like this
<picture_arm_startup_before_go>

The Robot Emergency Stop in the red frame indicates that the robot is currently E-stopped. In order to activate and use the arms E-stop must be turned off. There are multiple methods the robot could be in an E-stopped state. The red button on the back panel of the robot could be pressed. To deactivate that buttons E-stop, simply twist the button in a clockwise manner until it stops. This will turn off that buttons e-stop, however the robot may still be e-stopped. The robot could also be E-stopped via the controller. In order to release E-stop on the controller you need to press the buton on the controller labeld **GO** if the label has fallen off it is the top left black button. This will cancel out E-stop.

<controller_picture>


Once the robot is no longer in an E-stopped state, the UI should look like this:
<picture_of_ui_before_on>

**Click ON**

Half way through the bootup process of the arm this will come up:

<picture_pos_verfiy>

Take a look at the robot arm and verify that it is in the correct orientation as depicted in the visual model, then proceed to check the box on the left had side stating **I confirm that this shows the real robot pose**

Then **press Robot Position Verified** found on the bottom right side of UI

The UI should now look like this
<Picture_of_UI_halfway_done>

**Press Start**

The UI should look like this with all the process bubbles shown green for a successfull boot up

**You need to do this process for both arms**

##### Switching UI from one arm to the other
On the back panel where the left and right power buttons for the arms were, on the bottom left of that panel there is a button

##### Repeat process

Follow back through the steps provided in [Working with UI and controller to activate the arms](https://github.com/DiceLabs/Husky/blob/developed_ROS_packages/README.md#working-with-ui-and-controller-to-activate-the-arms)


SSH into robot dual bringup launch

grippers open/close

hit play button 

launch moveit

then RVIZ

## RVIZ

**after following []()** the robot model is fully ready for the startup of the visualization tool **RVIZ**














# Object Perception and navigation
The `ml_test` package enables object perception functionality, as well as navigation to perceived objects. As of now, the navigation script cannot handle dynamic objects, or objects which are not part of the originally generated map. A modified script will be made to try to fix this issue.


# Robot arm manipulation
The `test` package enables robot arm end-effector-to-perceived object interaction. The script in this package also handles final orientation of the end effector so it correctly interacts with object. This package has recently been tested with "bottle" objects.
