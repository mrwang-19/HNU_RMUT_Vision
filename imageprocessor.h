#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <QObject>
#include <QThread>
#include <QString>
#include <QMetaType>
#include <QVector>
#include <QMutex>
#include <QFuture>
#include <QtConcurrent>
#include <string>


#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"
using namespace cv;
using namespace std;

/**
 * @brief The Target struct 存放每帧识别到的能量机关信息
 */
struct Target
{
    uint64 index;               //序号
    bool hasTarget=true;        //是否含有有效目标
    char lightNum;              //已点亮的灯条数
    RotatedRect armorRect;      //要击打的装甲板的边缘矩形
    Point2f armorCenter;        //要击打的装甲板的中心像素坐标
    Point2f normalizedCenter;   //目标装甲板减去能量机关中心后的坐标
    float armorAngle=0;         //锤子角度(单位：deg)
    float angleDifference=0;    //间隔tao帧的角度差(单位：deg)
    Point2f center;             //能量机关中心坐标
    uint64_t timestamp;         //采集时间戳
    String toString();
};
Q_DECLARE_METATYPE(Target)

/**
 * @brief The ImageProcessor class 图像处理及记录结果
 */
class ImageProcessor : public QObject
{
    Q_OBJECT
public:
    explicit ImageProcessor(uint16_t height,uint16_t width,uint16_t frameRate,double blueDecay,QObject *parent = nullptr);
    ~ImageProcessor();
    uint16_t height,width;          //图像高宽
    uint16_t frameRate;             //帧率
    double blueDecay;               //蓝色衰减
    bool recordingFlag=false;       //录制标记
    float t;                        //规则时间
    bool rotateDirection=false;     //旋转方向，true顺时针，false逆时针
    int tao=75;                     //计算角度差的间隔帧数
    QVector<Mat> frameQueue;        //帧队列
    QMutex frameLock;               //帧队列锁
    QVector<Target> historyTarget;  //历史目标队列
    QMutex historyLock;             //历史目标队列锁
    QThreadPool processors;         //消费者线程池

public slots:
    void startRecording(QString savePath);
    void stopRecording();
    void onNewImage(char* img_data,int height,int width);
    void onNewImage(Mat Frame);
private:
    //函数
    Mat pretreatment(Mat frame);
    void detectTarget(uint64_t timestamp);

    //变量
    std::ofstream *csv_save=nullptr;    //保存的csv文件
    cv::VideoWriter *recorder =nullptr; //录制视频的句柄

signals:
    void newTarget(Target target);
};


#endif // IMAGEPROCESSOR_H
