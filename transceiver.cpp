#include "transceiver.h"
#include <QDebug>

Transceiver::Transceiver(QString portName, QObject *parent) : QObject(parent)
{    
//    memset(&recvFrame,0,sizeof (RecvFrame));
//    memset(&sendFrame,0,sizeof (SendFrame));
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
    qDebug()<<sendFrame.pitchAngleSet<<","<<sendFrame.yawAngleSet;
    serial->write((char*)&sendFrame,sizeof (SendFrame));
    serial->flush();
}

void Transceiver::receiveData()
{
    if(serial->bytesAvailable()>=(qint64)sizeof (RecvFrame))
    {
        serial->read((char*)&recvFrame,(qint64)sizeof (RecvFrame));
        serial->clear(QSerialPort::Input);
    }
    if(recvFrame.head!=0xbbbb)
        memset(&recvFrame,0,sizeof (RecvFrame));
}
Transceiver::~Transceiver()
{
    if(serial!=nullptr)
        serial->close();
}
