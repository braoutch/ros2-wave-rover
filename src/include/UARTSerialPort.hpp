#pragma once
#include <QString>;
#include <QSerialPort>

class UARTSerialPort {
    public:
        UARTSerialPort(QString path, int baudrate);
        ~UARTSerialPort();

    private:
        void sendRequest(const QString& text);
        void readResponse();

        QSerialPort _serial;
        QByteArray _response;
};

