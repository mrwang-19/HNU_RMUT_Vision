#include <QDateTime>
#include "chartpainter.h"

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
    addGraph(); // blue line
    graph(0)->setPen(QPen(QColor(40, 110, 255)));
    graph(0)->setLineStyle(QCPGraph::lsNone);
    graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
    addGraph(); // red line
    graph(1)->setPen(QPen(QColor(255, 110, 40)));
    graph(1)->setLineStyle(QCPGraph::lsNone);
    graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");
    xAxis->setTicker(timeTicker);
    axisRect()->setupFullAxesBox();
    yAxis->setRange(-1.2, 1.2);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(xAxis, SIGNAL(rangeChanged(QCPRange)), xAxis2, SLOT(setRange(QCPRange)));
    connect(yAxis, SIGNAL(rangeChanged(QCPRange)), yAxis2, SLOT(setRange(QCPRange)));
}

void ChartPainter::onTarget(Target target)
{
    if(target.hasTarget)
    {
        static QTime timeStart = QTime::currentTime();
        // calculate two new data points:
        double key = timeStart.msecsTo(QTime::currentTime())/1000.0; // time elapsed since start of demo, in seconds
        static double lastPointKey = 0;
        if (key-lastPointKey > 0.002) // at most add point every 2 ms
        {
          // add data to lines:
          graph(0)->addData(key, target.armorAngle);
          graph(1)->addData(key, target.angleDifference);
          // rescale value (vertical) axis to fit the current data:
          graph(0)->rescaleValueAxis();
          graph(1)->rescaleValueAxis(true);
          lastPointKey = key;
        }
        // make key axis range scroll with the data (at a constant range size of 8):
        xAxis->setRange(key, 8, Qt::AlignRight);
    }
}
