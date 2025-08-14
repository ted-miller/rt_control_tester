xhost +
sudo docker run --device=/dev/input/js0:/dev/input/js0 -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/ted:/home/ted --net=host osrf/ros:jazzy-desktop-full

source /opt/ros/jazzy/setup.bash

cd /home/ted/motoros2_moveit2_config_packages
rosdep update
rosdep install -i --from-path src -y
source install/setup.bash

ros2 run rt_control rt_control_node --ros-args -p robot_ip:=192.168.1.31