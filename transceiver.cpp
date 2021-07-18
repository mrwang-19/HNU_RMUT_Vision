#include "transceiver.h"
#include <QDebug>
#include <QTime>
#include <QThread>
#include <QByteArray>
using namespace std;

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
    timerID=startTimer(10ms);
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
    {
        count++;
        //打印时间戳
//        qDebug()<<QThread::currentThread()<<QTime::currentTime();
    }
    //最多发2次
    if(count>3)
    {
        count=0;
        sendFrame.shootCommand=0;
    }
}

void Transceiver::receiveData()
{
    if(serial->bytesAvailable()>=(qint64)sizeof(RecvFrame))
    {
        auto tmp=serial->read(2);
        if(tmp==QByteArray("\xbb\xbb",2))
        {
            serial->read(((char*)&recvFrame)+2,((qint64)sizeof (RecvFrame))-2);
//            if(recvFrame.shootStatusGet)
//                sendFrame.shootCommand=0;
        }
        else
        {
                qDebug()<<serial->read(1);
        }
    }
}
