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
//    float predictTime;          //预测时间
    float tao;                  //观测时间
    int samples;                //样本数量
    double phi[1]={0.0};        //拟合得到的相位
    int timerID;                //定时器ID
    uint64 startTimestamp;
    Point2f predictPoint(float predictTime);   //计算预测角度差；
private:
    ImageProcessor *processor;  //图像处理类
    ceres::Problem *problem;     //Ceres库待求解问题
    ceres::Solver::Options options;
protected:
    void timerEvent(QTimerEvent *e);

signals:
    void newTao(double timestamp,float tao);
};

#endif // PREDICTOR_H
