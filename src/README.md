# Source Code for Husky Operation
ROS APIs for each relevant component of the robot can be found here. We will use these components in tandem to perfrom the desired goals of the robot. 

- `components/`: Contains Source code for various parts of the robot such as arms, grippers, etc...
- `integration/`: Has code to tie together the use of multiple components at the same time
- `msgs/`: Holds custom ROS messages needed for communication with the arms 
- `utility/`: Contains different helpful mini "libraries" for common operations, but that isn't a literal part of the robot functionality
- `CMakeLists.txt/`: This is needed for colcon to build the custom messages