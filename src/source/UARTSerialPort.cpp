#include <UARTSerialPort.hpp>
#include <iostream>
#include <QSerialPortInfo>
#include <QDebug>
#include <thread>
#include <chrono>
#include <QTime>

UARTSerialPort::UARTSerialPort(QString path, int baudrate) {
    const auto infos = QSerialPortInfo::availablePorts();
    std::cout << "Detected Serial port:";
    for (const QSerialPortInfo &info : infos)
    {
        std::cout << "\n" << info.portName().toStdString() << std::endl;

        if(path == "") {
            path = info.portName();
            path = "/dev/" + info.portName();
        }
    }

    std::cout << (infos.size() ? "----" : " None.") << std::endl;
    qDebug() << "Opening " << path << " at baudrate " << baudrate << "...";

    _serial.setPortName(path);
    // _serial.setDataBits(QSerialPort::Data8);
    _serial.setBaudRate(baudrate);
    // _serial.setParity(QSerialPort::NoParity);
    // _serial.setStopBits(QSerialPort::OneStop);
    // _serial.setFlowControl(QSerialPort::NoFlowControl);

    if (!_serial.open(QIODevice::OpenModeFlag::ReadWrite))
    {
        qDebug() << (QString("Can't open %1, error code %2 : %3")
                     .arg(_serial.portName())
                     .arg(_serial.error())
                     .arg(_serial.errorString()));
        qDebug() << "Serial status: " << _serial.isOpen();

        exit(0);
    }

    QString command = "Hello You !";
    sendRequestSync(command);
//    QString response;
//    bool b = getResponseSync("Hello you ?", response);
//    qDebug() << b << " " << response;

}

UARTSerialPort::~UARTSerialPort() {
    _serial.close();
}

bool UARTSerialPort::sendRequestSync(const QString& text)
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    if(!isAvailable())
    {
        qDebug() << "Serial Port is down!";
        return false;
    }
    _serial.write((text + "\r\n").toUtf8());
    return _serial.waitForBytesWritten();
}

bool UARTSerialPort::getResponseSync(const QString& command, QString& response)
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    if(!isAvailable())
    {
        qDebug() << "Serial Port is down!";
        return false;
    }

    _serial.write((command + "\r\n").toUtf8());
    if (_serial.waitForBytesWritten(_writeTimeout)) {
        // read response
        if (_serial.waitForReadyRead(_receiveTimeout)) {
            QByteArray responseData = _serial.readAll();
            while (_serial.waitForReadyRead(10))
                responseData += _serial.readAll();

            response = QString::fromUtf8(responseData);
            qDebug() << "Message received: " << response;
            return true;

        } else {
            qDebug() << QString("Wait read response timeout %1")
                        .arg(QTime::currentTime().toString());
        }
    } else {
        qDebug() << QString("Wait write request timeout %1")
                    .arg(QTime::currentTime().toString());
    }
    return false;
}

bool UARTSerialPort::sendRequestSync(const QByteArray& text)
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    _serial.write(text);
    return _serial.waitForBytesWritten();
}

bool UARTSerialPort::readResponse()
{
    _response.append(_serial.readAll());
    return false;
}

bool UARTSerialPort::isAvailable()
{
    if(_serial.isOpen())
        return true;

    else
    {
        qDebug() << "Trying to recover the Serial Port...";
        _serial.close();
        return _serial.open(QIODevice::ReadWrite);
    }
    return false;
}


