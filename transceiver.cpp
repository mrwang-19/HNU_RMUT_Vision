#include "transceiver.h"
#include <QDebug>

Transceiver::Transceiver(QString portName, QObject *parent) : QObject(parent)
{    
    memset(&recvFrame,0,sizeof (RecvFrame));
    memset(&sendFrame,0,sizeof (RecvFrame));
    serial=new QSerialPort(portName,this);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setReadBufferSize(sizeof(RecvFrame));
    connect(serial,&QSerialPort::readyRead,this,&Transceiver::receiveData);
    serial->open(QIODevice::ReadWrite);
    startTimer(20);
}
void Transceiver::timerEvent(QTimerEvent *)
{
    serial->write((char*)&sendFrame,sizeof (SendFrame));
}

void Transceiver::receiveData()
{
    if(serial->bytesAvailable()>=(qint64)sizeof (RecvFrame))
    {
        serial->read((char*)&recvFrame,(qint64)sizeof (RecvFrame));
        serial->clear(QSerialPort::Input);
//        qDebug()<<recvFrame.head;
    }
    if(recvFrame.head!=0xbbbb)
        memset(&recvFrame,0,sizeof (RecvFrame));
}
