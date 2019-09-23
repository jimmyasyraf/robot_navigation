# ROS Robot Navigation
An overkill Arduino obstacle avoidance robot using ROS

## Setup
1. Initialize catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/jimmyasyraf/robot_navigation.git
```
2. Build the package
```
cd ~/catkin_ws
catkin_make
```
3. Run ROS core
```
roscore
```

4. Run rosserial to establish serial connection with Arduino. Make sure Arduino port is correct
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

5. Run the package
```
rosrun robot_navigation obstacle_avoidance.py
```

6. View logs
```
rostopic echo /distance
rostopic echo /direction
```
