#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <QObject>
#include <QThread>
#include <QString>
#include <QTime>

#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
///
/// \brief The ImageProcessor class
/// 图像处理及记录结果
///
class ImageProcessor : public QObject
{
    Q_OBJECT
public:
    explicit ImageProcessor(uint16_t height,uint16_t width,uint16_t frameRate,QObject *parent = nullptr);
    uint16_t height,width;      //图像高宽
    uint16_t frameRate;         //帧率
    bool recordingFlag=false;   //录制标记
    Mat *orignalImage=nullptr;  //原始图片
    Mat *binaryImage=nullptr;   //二值化图片
    QTime t;                    //规则时间

public slots:
    void startRecording(QString savePath);
    void stopRecording();
    void onNewImage(char* img_data,int height,int width);
private:
    //函数
    void pretreatment(Mat *frame);
    bool detectTarget(Point2f &center,Point2f &armor);
    //变量
    std::ofstream *csv_save=nullptr;    //保存的csv文件
    cv::VideoWriter *recorder =nullptr; //录制视频的句柄

signals:

};
/**
 * @brief The Target struct 存放每帧识别到的能量机关信息
 */
struct Target
{
    int index;          //序号
    bool hasTarget;     //是否含有有效目标


}

#endif // IMAGEPROCESSOR_H
