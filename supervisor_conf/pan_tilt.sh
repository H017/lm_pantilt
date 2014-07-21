#!/bin/bash

source /home/ubuntu/ros_catkin_ws/install_isolated/setup.bash

export IBEX_HOME=/home/capra/Ibex

if [ -f $IBEX_HOME/devel/setup.bash ]; then
    source $IBEX_HOME/devel/setup.bash
fi

export ROS_MASTER_URI=http://192.168.7.1:11311/
export ROS_IP=192.168.7.2

while true
do
    roslaunch $(rospack find lm_pantilt)/launch/PTU.launch
    sleep 5
done