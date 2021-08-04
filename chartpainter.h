#ifndef CHARTPAINTER_H
#define CHARTPAINTER_H

#include <QObject>
#include "qcustomplot/qcustomplot.h"

#include "imageprocessor.h"

class ChartPainter : public QCustomPlot
{
    Q_OBJECT
public:
    explicit ChartPainter(QWidget *parent = nullptr);
public slots:
    void onTarget(Target target);
    void onAngleDifference(uint64_t timestamp,float angleDifference);
    void onPhi(uint64 timestamp,float phi);
    void onSpeed(uint64 timestamp,float speed);
private:
    uint64 timeStart = QDateTime::currentDateTime().toMSecsSinceEpoch();

signals:


};

#endif // CHARTPAINTER_H
