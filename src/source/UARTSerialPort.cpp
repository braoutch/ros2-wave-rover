#include <UARTSerialPort.hpp>
#include <iostream>
#include <QSerialPortInfo>
#include <QDebug>

UARTSerialPort::UARTSerialPort(QString path, int baudrate) {
    const auto infos = QSerialPortInfo::availablePorts();
    std::cout << "Detected Serial port:";
    for (const QSerialPortInfo &info : infos)
        std::cout << "\n" << info.portName().toStdString() << std::endl;
    std::cout << (infos.size() ? "----" : " None.") << std::endl;

    _serial.setPortName(path);

    if (!_serial.open(QIODevice::ReadWrite))
    {
        qDebug() << (QString("Can't open %1, error code %2")
                         .arg(_serial.portName())
                         .arg(_serial.error()));
    }
}

UARTSerialPort::~UARTSerialPort() {
    _serial.close();
}

void UARTSerialPort::sendRequest(const QString& text)
{
    _serial.write(text.toUtf8());
    // m_timer.start(m_waitResponseSpinBox->value());
}

void UARTSerialPort::readResponse()
{
    _response.append(_serial.readAll());
}