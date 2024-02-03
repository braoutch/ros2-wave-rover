#include <RobotController.hpp>
#include <UARTSerialPort.hpp>
#include <json.hpp>
#include <RoverCommands.hpp>
#include <QDebug>

RobotController::RobotController() {
    _pUARTSerialPort = std::make_shared<UARTSerialPort>("/dev/pts/2", 9600);
}

RobotController::~RobotController() {
    
}

bool RobotController::SendCmdVel(geometry_msgs::msg::Twist cmd_vel){
    nlohmann::json message_json = {};

    return false;
}

bool RobotController::SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command, QString& response){
    nlohmann::json message_json = {};
    message_json["T"] = command;
    return _pUARTSerialPort->getResponseSync(QString::fromStdString(message_json.dump()), response);
}

bool RobotController::SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command){
    nlohmann::json message_json = {};
    message_json["T"] = command;
    return _pUARTSerialPort->sendRequestSync(QString::fromStdString(message_json.dump()));
}

bool RobotController::EnableWifiHotspot(){
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::WIFI_AP_DEFAULT);
}

bool RobotController::ScanWifi(){
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::WIFI_SCAN);
}

bool RobotController::DisableWifi(){
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::WIFI_OFF);
}


bool RobotController::EmergencyStop(){
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::EMERGENCY_STOP);
}

bool RobotController::SetOled(int row, QString content){
    if(row > 3)
    {
        qDebug() << "Cannot print anything on OLED row " << row;
        return false;
    }

    nlohmann::json message_json = {};
    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::OLED_SET;
    message_json["lineNum"] = row;
    message_json["Text"] = content.toStdString();
    return _pUARTSerialPort->sendRequestSync(QString::fromStdString(message_json.dump()));
}

bool RobotController::ResetOled() {
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::OLED_DEFAULT);
}

bool RobotController::GetInformation(INFO_TYPE info_type, QString& response){
    WAVE_ROVER_COMMAND_TYPE target_type;
    switch (info_type)
    {
    case INFO_TYPE::IMU:
        target_type = WAVE_ROVER_COMMAND_TYPE::IMU_INFO;
        break;

    case INFO_TYPE::DEVICE:
        target_type = WAVE_ROVER_COMMAND_TYPE::DEVICE_INFO;
        break;

    case INFO_TYPE::INA219:
        target_type = WAVE_ROVER_COMMAND_TYPE::INA219_INFO;
        break;

    case INFO_TYPE::WIFI:
        target_type = WAVE_ROVER_COMMAND_TYPE::WIFI_INFO;
        break;
    }
    return SendGenericCmd(target_type, response);
}

