# ros2-wave-rover

## Requirements
```
sudo apt install -y qtcreator qtbase5-dev qt5-qmake cmake libqt5serialport5-dev
```

## Create a serial port, for testing purpose
```
socat -v -d -d PTY,raw,echo=0,b115200,cs8 PTY,raw,echo=0,b115200,cs8
```