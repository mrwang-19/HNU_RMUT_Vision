#include "transceiver.h"
#include <QDebug>

Transceiver::Transceiver(QString portName, QObject *parent) : QObject(parent)
{    
    serial=new QSerialPort(portName);
    serial->setStopBits(QSerialPort::OneStop);
}
