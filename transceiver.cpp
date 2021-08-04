#include "transceiver.h"
#include <QDebug>
#include <QTime>
#include <QThread>
#include <QByteArray>
using namespace std;

Transceiver::Transceiver(QString portName, QObject *parent) : QObject(parent)
{
    serial=new QSerialPort(portName,this);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setReadBufferSize(sizeof(RecvFrame));
    //注册串口接收回调函数
    connect(serial,&QSerialPort::readyRead,this,&Transceiver::receiveData);
    serial->open(QIODevice::ReadWrite);
    //串口发送帧率100Hz
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
    if(serial->isOpen())
    {
        serial->write((char*)&sendFrame,sizeof (SendFrame));
        serial->flush();
    }
    if(sendFrame.shootCommand)
    {
        count++;
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
        }
        else
        {
                qDebug()<<serial->read(1);
        }
    }
}
