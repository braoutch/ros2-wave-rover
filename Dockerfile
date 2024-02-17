FROM arm64v8/ros:humble-ros-base
# FROM ros:humble-ros-base
RUN apt update && apt install -y git qtbase5-dev qt5-qmake cmake libqt5serialport5-dev nano && apt clean && rm -rf /var/lib/apt/lists/
RUN git clone https://github.com/braoutch/ros2-wave-rover

WORKDIR /ros2-wave-rover

RUN echo "2" && . /opt/ros/humble/setup.sh && colcon build

# Setup environment variables
COPY entrypoint.sh /sbin/entrypoint.sh
RUN sudo chmod 755 /sbin/entrypoint.sh

ENTRYPOINT ["/sbin/entrypoint.sh"]
# CMD ["bash"]
