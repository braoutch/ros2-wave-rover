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

## Give the rights to the serial port
The easy way:

sudoedit /etc/udev/rules.d/50-myusb.rules

Save this text:
```
KERNEL=="ttyUSB[0-9]*",MODE="0666"
KERNEL=="ttyACM[0-9]*",MODE="0666"
```
Then unplug and replug the device.