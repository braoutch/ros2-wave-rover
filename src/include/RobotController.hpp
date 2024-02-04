#pragma once
#include <QString>
#include <memory>
#include <thread>
#include <RoverCommands.hpp>
#include <rclcpp/rclcpp.hpp>

// cmd_vel message type
#include <geometry_msgs/msg/twist.hpp>

class UARTSerialPort;
class ROS2Subscriber;

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
        bool SendCmdVel(geometry_msgs::msg::Twist::SharedPtr cmd_vel);

    private:
        std::shared_ptr<UARTSerialPort> _pUARTSerialPort;

        std::shared_ptr<ROS2Subscriber> _pROS2Subscriber;
        std::unique_ptr<std::thread> _execThread;
        rclcpp::Executor::SharedPtr _executor;
        void RunRos2Exectutor();

        bool SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command, QString& result);
        bool SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command);
};

