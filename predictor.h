#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <QObject>
#include <QDebug>
#include <ceres/ceres.h>
#include <cmath>

#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "imageprocessor.h"

using namespace cv;
using namespace std;

class Predictor : public QObject
{
    Q_OBJECT
public:
    explicit Predictor(ImageProcessor * processor,int samples,QObject *parent = nullptr);
    ~Predictor();
//    float predictTime;        //预测时间
    uint tau;                //计算角度差的间隔帧数
    float tao;                  //观测时间
    int samples;                //样本数量
    double phi;                 //拟合得到的相位
    double last_phi;            //上一次拟合得到的相位
    int timerID;                //定时器ID
    uint64 startTimestamp;      //起始帧时间辍
    Target currentTarget;
    Point2f predictPoint(float predictTime);   //计算预测角度差；
    float getSpeed(float predictTime);
private:
    ImageProcessor *processor;      //图像处理类
    ceres::Problem *problem;        //Ceres库待求解问题
    ceres::Solver::Options options;
//    QVector<Target> list;           //processor->historyTarget的拷贝
protected:
    void timerEvent(QTimerEvent *e);

signals:
    void newPhi(uint64_t timestamp,float phi);
    void newSpeed(uint64_t timestamp,float phi);
};

#endif // PREDICTOR_H
