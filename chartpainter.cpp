#include <QDateTime>
#include "chartpainter.h"
#define REAL_ENERGY

ChartPainter::ChartPainter(QWidget *parent) : QCustomPlot(parent)
{    
    // include this section to fully disable antialiasing for higher performance:
    /*
    setNotAntialiasedElements(QCP::aeAll);
    QFont font;
    font.setStyleStrategy(QFont::NoAntialias);
    xAxis->setTickLabelFont(font);
    yAxis->setTickLabelFont(font);
    legend->setFont(font);
    */
    addGraph(); // 深蓝线，当前目标角度
    graph(0)->setPen(QPen(QColor(0, 0, 0xff)));
    graph(0)->setLineStyle(QCPGraph::lsNone);
    graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
    addGraph(); // 浅蓝线，跳变后折算的角度
    graph(1)->setPen(QPen(QColor(0, 0xcc, 0xff)));
    graph(1)->setLineStyle(QCPGraph::lsNone);
    graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
    addGraph(); // 橙线，预测的角速度
    graph(2)->setPen(QPen(QColor(255, 110, 40)));
    graph(2)->setLineStyle(QCPGraph::lsNone);
    graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
    addGraph(); // 黑线，角度差
    graph(3)->setPen(QPen(QColor(0, 0, 0)));
    graph(3)->setLineStyle(QCPGraph::lsNone);
    graph(3)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
    addGraph(); // 绿线，拟合得到的相位
    graph(4)->setPen(QPen(QColor(0, 255, 0)));
    graph(4)->setLineStyle(QCPGraph::lsNone);
    graph(4)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%s");
    timeTicker->setTickCount(4);
    xAxis->setTicker(timeTicker);
    axisRect()->setupFullAxesBox();
    yAxis->setRange(0, CV_2PI);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(xAxis, SIGNAL(rangeChanged(QCPRange)), xAxis2, SLOT(setRange(QCPRange)));
    connect(yAxis, SIGNAL(rangeChanged(QCPRange)), yAxis2, SLOT(setRange(QCPRange)));
}
/// onTarget 绘制当前角度和折算角度的槽函数
/// \param target 当前目标对象
void ChartPainter::onTarget(Target target)
{
    if(target.hasTarget)
    {
        // 目标对象内本就包含时间戳
        double key = (target.timestamp-timeStart)/1000000000.0; // time elapsed since start of demo, in seconds
        static double lastPointKey = 0;
        if (key-lastPointKey > 0.002) // at most add point every 2 ms
        {
          // 画角度
          graph(0)->addData(key, target.armorAngle);
          // 画折算角度
          graph(1)->addData(key, target.lastArmorAngle);

          // rescale value (vertical) axis to fit the current data:
          graph(0)->rescaleValueAxis();
          graph(1)->rescaleValueAxis(true);
          lastPointKey = key;
        }
        // make key axis range scroll with the data (at a constant range size of 8):
        xAxis->setRange(key, 8, Qt::AlignRight);
    }
}

/// onSpeed
/// \param timestamp
/// \param speed
void ChartPainter::onSpeed(uint64 timestamp,float speed)
{
    double key = (timestamp-timeStart)/1000000000.0;
    graph(2)->addData(key , speed);
}

/// onAngleDifference 绘制角度差的槽函数
/// \param timestamp 时间戳
/// \param angleDifference 角度差
void ChartPainter::onAngleDifference(uint64_t timestamp, float angleDifference)
{
    double key = (timestamp-timeStart)/1000000000.0;
    graph(3)->addData(key , angleDifference);
}

/// onPhi 绘制拟合相位的槽函数
/// \param timestamp 时间戳
/// \param phi 相位值
void ChartPainter::onPhi(uint64 timestamp,float phi)
{
    double key = (timestamp-timeStart)/1000000000.0;
    graph(4)->addData(key , phi);
}