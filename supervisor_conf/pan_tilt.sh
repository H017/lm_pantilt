#!/bin/bash

source /home/ubuntu/ros_catkin_ws/install_isolated/setup.bash

export IBEX_HOME=/home/capra/Ibex

if [ -f $IBEX_HOME/devel/setup.bash ]; then
    source $IBEX_HOME/devel/setup.bash
fi

export ROS_MASTER_URI=http://192.168.32.100:11311/
export ROS_IP=192.168.32.245

while true
do
    roslaunch $(rospack find lm_pantilt)/launch/PTU.launch
    sleep 5
done