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
    timerID=startTimer(20);
}
Transceiver::~Transceiver()
{
    killTimer(timerID);
    if(serial->isOpen())
        serial->close();
}
void Transceiver::timerEvent(QTimerEvent *)
{
//    qDebug()<<sendFrame.pitchAngleSet<<","<<sendFrame.yawAngleSet;
    if(serial->isOpen())
    {
        serial->write((char*)&sendFrame,sizeof (SendFrame));
        serial->flush();
    }
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
    //如果已发射则取消发射指令
    if(recvFrame.shootStatusGet)
        sendFrame.shootCommand=0;
}
