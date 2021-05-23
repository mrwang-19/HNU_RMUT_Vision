#include <QDateTime>
#include "chartpainter.h"

ChartPainter::ChartPainter(QWidget *parent) : QChartView(parent)
{
    QPen penY(Qt::darkBlue,3,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
    chart = new QChart();
    series = new QSplineSeries;
    axisX = new QDateTimeAxis();
    axisY = new QValueAxis();

    chart->legend()->hide();                             //隐藏图例
    chart->addSeries(series);                            //把线添加到chart
    axisX->setTickCount(10);                             //设置坐标轴格数
    axisY->setTickCount(6);
    axisX->setFormat("ss");                        //设置时间显示格式
    axisY->setMin(0);                                    //设置Y轴范围
    axisY->setMax(180);
    axisX->setTitleText("实时时间");                       //设置X轴名称
    axisY->setLinePenColor(QColor(Qt::darkBlue));        //设置坐标轴颜色样式
    axisY->setGridLineColor(QColor(Qt::darkBlue));
    axisY->setGridLineVisible(false);                    //设置Y轴网格不显示
    axisY->setLinePen(penY);
    axisX->setLinePen(penY);

    chart->addAxis(axisX,Qt::AlignBottom);               //设置坐标轴位于chart中的位置
    chart->addAxis(axisY,Qt::AlignLeft);
    chart->setBackgroundVisible(false);

    series->attachAxis(axisX);                           //把数据添加到坐标轴上
    series->attachAxis(axisY);

    axisY->setTitleText("角度差");

    //把chart显示到窗口上
    setChart(chart);
    setRenderHint(QPainter::Antialiasing);   //设置抗锯齿
}

void ChartPainter::onTarget(Target target)
{
    chart->axisX()->setMin(QDateTime::currentDateTime().addSecs(-60 * 1));       //系统当前时间的前一秒
    chart->axisX()->setMax(QDateTime::currentDateTime().addSecs(0));
    series->append(target.timestamp, target.angleDifference);
}
