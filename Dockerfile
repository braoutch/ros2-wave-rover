FROM docker pull ros:humble-ros-core
git clone https://github.com/braoutch/ros2-wave-rover
cd ros2-wave-rover
source /opt/ros/humble
colcon build