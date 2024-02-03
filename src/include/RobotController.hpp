#pragma once
#include <QString>
#include <memory>

#include <RoverCommands.hpp>

// cmd_vel message type
#include <geometry_msgs/msg/twist.hpp>

class UARTSerialPort;

class RobotController {
    public:
        RobotController();
        ~RobotController();

        bool EnableWifiHotspot();
        bool DisableWifi();
        bool ScanWifi();
        bool EmergencyStop();

        bool SetOled(int row, QString content);
        bool ResetOled();

        bool GetInformation(INFO_TYPE info_type, QString& response);

        bool SendCmdVel(geometry_msgs::msg::Twist cmd_vel);
        bool SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command, QString& result);
        bool SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command);


    private:
        std::shared_ptr<UARTSerialPort> _pUARTSerialPort;
};

