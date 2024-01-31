#include <RobotController.hpp>
#include <UARTSerialPort.hpp>

RobotController::RobotController() {
    _pUARTSerialPort = std::make_shared<UARTSerialPort>("", 1000000);
}

RobotController::~RobotController() {
    
}