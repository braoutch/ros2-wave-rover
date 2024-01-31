#pragma once
#include <QString>
#include <memory>

class UARTSerialPort;

class RobotController {
    public:
        RobotController();
        ~RobotController();

    private:
        std::shared_ptr<UARTSerialPort> _pUARTSerialPort;
};