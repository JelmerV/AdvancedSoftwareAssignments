# show jiwy image
ros2 run image_tools showimage --ros-args --remap image:=image_thres

# set threhold parameter
ros2 param set /jiwy_trajectory threshold  200


# initialize new node src (run in src folder)
ros2 pkg create --build-type ament_cmake --node-name my_node my_package

# Compile cpp program
g++ -o output file.cpp

#compile to use xenomai
g++ -o output file.cpp /usr/xenomai/bin/xeno-config --skin=posix --cflags --ldflags

