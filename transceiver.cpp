#include "transceiver.h"
#include <QDebug>

using namespace std;

Transceiver::Transceiver(QString portName, QObject *parent) : QObject(parent)
{    
//    memset(&recvFrame,0,sizeof (RecvFrame));
//    memset(&sendFrame,0,sizeof (SendFrame));
    serial=new QSerialPort(portName,this);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setBaudRate(QSerialPort::Baud115200);
//    serial->setReadBufferSize(sizeof(RecvFrame));
    connect(serial,&QSerialPort::readyRead,this,&Transceiver::receiveData);
    serial->open(QIODevice::ReadWrite);
    timerID=startTimer(20ms);
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
    if(sendFrame.shootCommand)
        count++;
    if(count>10)
    {
        count=0;
        sendFrame.shootCommand=0;
    }
}

void Transceiver::receiveData()
{
    if(serial->bytesAvailable()>=(qint64)sizeof (RecvFrame))
    {
        if(serial->peek(2).toInt()==0xbbbb)
        {
            serial->read((char*)&recvFrame,(qint64)sizeof (RecvFrame));
            if(recvFrame.shootStatusGet)
                sendFrame.shootCommand=0;
        }
        else
        {
            serial->read(1);
        }
    }
}
