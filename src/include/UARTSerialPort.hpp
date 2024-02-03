#pragma once
#include <QString>
#include <QSerialPort>
#include <QObject>
#include <mutex>
#include <thread>


class UARTSerialPort : public QObject {
    Q_OBJECT

    public:
        UARTSerialPort(QString path, int baudrate);
        ~UARTSerialPort();

        bool isAvailable();

        bool sendRequestSync(const QString& text);
        bool getResponseSync(const QString& command, QString& response);
        bool sendRequestSync(const QByteArray& text);

        bool readResponse();
        void readResponseThread();

    private:
        QSerialPort _serial;
        QByteArray _response;

        std::mutex _serial_mutex;

        std::thread _receive_thread;

        const int _receiveTimeout = 10000;
        const int _writeTimeout = 10000;

};


