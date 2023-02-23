#!/bin/bash

rosdep install -i --from-path src --rosdistro humble -y

colcon build
#colcon build --symlink-install

source /opt/ros/humble/setup.bash
source ./install/setup.bash