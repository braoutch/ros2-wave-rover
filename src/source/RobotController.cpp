#include <RobotController.hpp>
#include <UARTSerialPort.hpp>
#include <ROS2Subscriber.hpp>
#include <json.hpp>
#include <RoverCommands.hpp>
#include <QDebug>
#include <algorithm>
#include <QtConcurrent/QtConcurrent>
#include <JoypadController.hpp>

RobotController::RobotController() {
    // _pUARTSerialPort = std::make_shared<UARTSerialPort>("/dev/pts/10", 1000000);

    _pROS2Subscriber = std::make_shared<ROS2Subscriber>();
    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _execThread = std::make_unique<std::thread>(&RobotController::RunRos2Exectutor, this);

    int enable_joypad = 0;
    enable_joypad = _pROS2Subscriber->get_parameter("enable_joypad").as_int();
    std::string UART_address = _pROS2Subscriber->get_parameter("UART_address").as_string();

    qDebug() << "Joypad enabled: " << enable_joypad;
    qDebug() << "UART Address: " << QString::fromStdString(UART_address);

    _pROS2Subscriber->SubscribeToTopic("/cmd_vel", [&](const geometry_msgs::msg::Twist::SharedPtr msg){ SendCmdVel(msg); });

    _pUARTSerialPort = std::make_shared<UARTSerialPort>(QString::fromStdString(UART_address), 1000000); // open automatically the first serial port that is found
    if(enable_joypad)
    {
        _pJoypadController = std::make_shared<JoypadController>();
        // QObject::connect(_pJoypadController.get(), &JoypadController::JoypadCommandAvailable, [&](TimestampedDouble t1, TimestampedDouble t2){JoypadCommandReceived(t1, t2);});
        QObject::connect(_pJoypadController.get(), SIGNAL(JoypadCommandAvailable(TimestampedDouble, TimestampedDouble)), this, SLOT(JoypadCommandReceived(TimestampedDouble, TimestampedDouble)));
        _pJoypadController->start();
    }
    QString infos;
    GetInformation(INFO_TYPE::DEVICE, infos);
    DisplayMessage(5, "", "Gros Pote", "se réveille", "");
}

RobotController::~RobotController() {
    SendEmergencyStop();
    rclcpp::shutdown();
    _executor->cancel();
    if (_execThread->joinable()) {
        _execThread->join();
    }
    qDebug() << "ROS2 Node shut down.";
}

void RobotController::RunRos2Exectutor(){
    std::cout << "STARTING EXECUTOR" << std::endl;
    _executor->add_node(_pROS2Subscriber);
    _executor->spin();
    _executor->remove_node(_pROS2Subscriber);
}

bool RobotController::DisplayMessage(int seconds, QString line_1, QString line_2, QString line_3, QString line_4){
    ResetOled();
    SetOled(0, line_1);
    SetOled(1, line_2);
    SetOled(2, line_3);
    SetOled(3, line_4);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    return ResetOled();
}

bool RobotController::DisplayRollingMessage(QString line){
    ResetOled();
    _last_current_debug_row ++;
    if(_last_current_debug_row > 3) _last_current_debug_row = 0;
    return SetOled(_last_current_debug_row, line);

}

bool RobotController::SendCmdVel(geometry_msgs::msg::Twist::SharedPtr msg){
    nlohmann::json message_json = {};
    float l = 0, r = 0;

    // Cap values at [-1 .. 1]
    float x = std::max(std::min((double)msg->linear.x, 1.0), -1.0);
    float z = std::max(std::min((double)msg->angular.z, 1.0), -1.0);

    // Calculate the intensity of left and right wheels. Simple version.
    // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
    l = (x - z) / 2;
    r = (x + z) / 2;

    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::SPEED_INPUT;
    message_json["L"] = l;
    message_json["R"] = r;
    return _pUARTSerialPort->sendRequestSync(QString::fromStdString(message_json.dump()));
}

void RobotController::JoypadCommandReceived(TimestampedDouble t1, TimestampedDouble t2) {
    qDebug() << "Joypad command received";
    std::shared_ptr<geometry_msgs::msg::Twist> msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = t1.value;
    msg->angular.z = t2.value;

    SendCmdVel(msg);
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

bool RobotController::SendEmergencyStop() {
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::EMERGENCY_STOP);
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
