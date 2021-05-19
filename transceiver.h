#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>

class Transceiver : public QObject
{
    Q_OBJECT
public:
    explicit Transceiver(QString portName, QObject *parent = nullptr);

signals:

private:
    QSerialPort * serial=nullptr;
};

#endif // TRANSCEIVER_H
