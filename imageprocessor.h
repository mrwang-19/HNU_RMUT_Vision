#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <QObject>
#include <QThread>
#include <QString>
//#include <QTime>
#include <QVector>
#include <string>
#include <ctime>
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
    int index;                  //序号
    bool hasTarget;             //是否含有有效目标
    char lightNum;              //已点亮的灯条数
    RotatedRect armorRect;      //要击打的装甲板的边缘矩形
    Point2f armorCenter;        //要击打的装甲板的中心像素坐标
    Point2f normalizedCenter;   //目标装甲板减去能量机关中心后的坐标
    float armorAngle=0;           //锤子角度(单位：deg)
    float angleDifference=0;      //间隔tao帧的角度差(单位：deg)
    Point2f center;             //能量机关中心坐标
    uint64_t timestamp;            //采集时间戳
    String toString();
};

/**
 * @brief The ImageProcessor class 图像处理及记录结果
 */
class ImageProcessor : public QObject
{
    Q_OBJECT
public:
    explicit ImageProcessor(uint16_t height,uint16_t width,uint16_t frameRate,double blueDecay,QObject *parent = nullptr);
    uint16_t height,width;          //图像高宽
    uint16_t frameRate;             //帧率
    double blueDecay;               //蓝色衰减
    bool recordingFlag=false;       //录制标记
    Mat *orignalImage=nullptr;      //原始图片
    Mat *binaryImage=nullptr;       //二值化图片
    float t;                        //规则时间
    bool rotateDirection=true;            //旋转方向，true顺时针，false逆时针
    int tao=50;                     //计算角度差的间隔帧数
    QVector<Target> historyTarget;  //历史目标队列

public slots:
    void startRecording(QString savePath);
    void stopRecording();
    void onNewImage(char* img_data,int height,int width);
private:
    //函数
    void pretreatment(Mat *frame);
    Target detectTarget(uint64_t timestamp);

    //变量
    std::ofstream *csv_save=nullptr;    //保存的csv文件
    cv::VideoWriter *recorder =nullptr; //录制视频的句柄

signals:

};


#endif // IMAGEPROCESSOR_H
