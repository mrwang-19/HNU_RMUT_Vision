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

//TODO 确定历史长度,暂定500组
#define HISTORY_LENGTH 500

using namespace cv;
using namespace std;

/**
 * @brief The Target struct 存放每帧识别到的能量机关信息
 */
struct Target
{
    uint64 index;               //序号
    bool hasTarget=false;       //是否含有有效目标
    char lightNum;              //已点亮的灯条数
    RotatedRect armorRect;      //要击打的装甲板的边缘矩形
    Point2f armorCenter;        //要击打的装甲板的中心像素坐标
    Point2f normalizedCenter;   //目标装甲板减去能量机关中心后的坐标
    float armorAngle=0;         //锤子角度(单位：rad)
    bool jumpFlag=false;        //跳变分界帧标记
    float lastArmorAngle=0;     //折算至上一次跳变前的锤子角度(单位：rad)
    Point2f center;             //能量机关中心坐标
    uint64_t timestamp;         //采集时间戳
    float radius;               //半径
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
    explicit ImageProcessor(uint16_t height,uint16_t width,uint16_t frameRate,QObject *parent = nullptr);
    ~ImageProcessor();
    uint16_t height,width;                  //图像高宽
    uint16_t frameRate;                     //帧率

    static double blueDecay;                //蓝色衰减
    static uint8_t binaryThreshold;         //二值化阈值
    static uint8_t dilateKernelSize;        //膨胀核大小
    static uint8_t maxRadius,minRadius;     //最大最小半径
    static uint8_t rRadius;       //中心R logo半径

    bool recordingFlag=false;               //录制标记
    float t;                                //规则时间
    bool rotateDirection=true;              //旋转方向，true顺时针，false逆时针
    float lastJumpAngle;                    //上次跳变的角度，用于折算
    int indexOfLastJump;                    //targetHistory中上次跳变分界帧的索引
    QVector<Mat> frameQueue;                //帧队列
    QMutex frameLock;                       //帧队列锁
    QVector<Target> historyTarget;          //历史目标队列
    QMutex historyLock;                     //历史目标队列锁
    QThreadPool processors;                 //消费者线程池

public slots:
    void startRecording(QString savePath);
    void stopRecording();
    void onNewImage(char* img_data,int height,int width,uint64_t timestamp);
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
