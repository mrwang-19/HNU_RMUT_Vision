#ifndef CHARTPAINTER_H
#define CHARTPAINTER_H

#include <QObject>
#include <QtCharts/QChartView>
#include <QtCharts/QSplineSeries>
#include <QDateTimeAxis>
#include <QValueAxis>

#include "imageprocessor.h"

QT_CHARTS_USE_NAMESPACE   //使用QChart必须要添加这句

class ChartPainter : public QChartView
{
    Q_OBJECT
public:
    explicit ChartPainter(QWidget *parent = nullptr);
public slots:
    void onTarget(Target target);
private:
    QChart *chart;                           //画布
    QSplineSeries *series;                     //线
    QDateTimeAxis *axisX;                    //轴
    QValueAxis *axisY;

signals:


};

#endif // CHARTPAINTER_H
