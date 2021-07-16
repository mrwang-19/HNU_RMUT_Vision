#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>

struct __attribute__((packed)) SendFrame
{
    uint16_t head=0xaaaa;   //帧头
    float pitchAngleSet=0.0f;    //pitch轴角度设定值
    float yawAngleSet=0.0f;      //yaw轴角度设定值
    float targetAngle=0.0f;      //目标装甲板角度
    uint8_t shootCommand=0;   //发射指令
};

struct __attribute__((packed)) RecvFrame
{
    uint16_t head=0xbbbb;   //帧头
    float pitchAngleGet;    //pitch轴角度设定值
    float yawAngleGet;      //yaw轴角度设定值
    uint8_t rotateDricetion;   //旋转方向
    float timeBais;         //预测时间偏置
    float compensateBais;   //弹道补偿偏置
    int shootStatusGet;     //发射反馈
};


class Transceiver : public QObject
{
    Q_OBJECT
public:
    explicit Transceiver(QString portName, QObject *parent = nullptr);
    ~Transceiver();
    SendFrame sendFrame;
    RecvFrame recvFrame;
    int timerID;
    uint count;     //shootcmd计数

signals:

private:
    QSerialPort * serial=nullptr;
private slots:
    void receiveData();
protected:
    void timerEvent(QTimerEvent *e);
};

#endif // TRANSCEIVER_H
