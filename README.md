# ros2-wave-rover

## Requirements
```
sudo apt install -y qtcreator qtbase5-dev qt5-qmake cmake libqt5serialport5-dev
```

## Create a serial port, for testing purpose
```
socat -v -d -d PTY,raw,echo=0,b115200,cs8 PTY,raw,echo=0,b115200,cs8
```

## For the real one, to test it:
```
stty -F /dev/ttyUSB0 1000000 # set the baud rate
cat /dev/ttyUSB0 # will receive commands that are sent from the robot
echo -ne "{\"T\":-3}" > /dev/ttyUSB0 # will send commands
```

## To create the udev rule
```
sudo cp 99-waverover.rules /etc/udev/rules.d/99-waverover.rules
```


## Give the rights to the serial port
The easy way:

sudoedit /etc/udev/rules.d/50-myusb.rules

Save this text:
```
KERNEL=="ttyUSB[0-9]*",MODE="0666"
KERNEL=="ttyACM[0-9]*",MODE="0666"
```
Then unplug and replug the device.

# Run the docker image
```
docker run -it --rm  braoutch/ros2-wave-rover ros2 launch gros-pote gros-pote-node
```
or
```
docker run -it --rm  braoutch/ros2-wave-rover  ros2 launch gros-pote wave_rover_launch.py enable_joypad:=0 UART_address="DUMMY"
```
It will also run (slowly) on x86 if you ran the qemu line before.

# Docker build - raspberry pi

## QEMU
```
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

### Alternative
```
docker buildx create --use
docker buildx build --platform linux/arm64 -t braoutch/ros2-wave-rover:latest --push .
```