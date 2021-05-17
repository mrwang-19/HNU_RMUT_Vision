#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <QObject>

class Transceiver : public QObject
{
    Q_OBJECT
public:
    explicit Transceiver(QObject *parent = nullptr);

signals:

};

#endif // TRANSCEIVER_H
