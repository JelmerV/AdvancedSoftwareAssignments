# show jiwy image
ros2 run image_tools showimage --ros-args --remap image:=image_thres

# set threhold parameter
ros2 param set /jiwy_trajectory threshold  200