FROM arm64v8/ros:humble-ros-base
RUN apt update && apt install -y git qtbase5-dev qt5-qmake cmake libqt5serialport5-dev
RUN git clone https://github.com/braoutch/ros2-wave-rover
RUN cd ros2-wave-rover && . /opt/ros/humble/setup.sh && colcon build