#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <QObject>
#include <QThread>
#include <QString>

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
    uint16_t height,width;
    uint16_t frameRate;
    bool recordingFlag=false;           //录制标记

private slots:
    void startRecording(QString savePath);
    void stopRecording();
private:
    //函数
    Mat pretreatment(Mat frame);
    bool detectTarget(Mat src,Point2f &center,Point2f &armor);
    void onNewImage(char* img_data,int height,int width);
    //变量
    std::ofstream *csv_save=nullptr;    //保存的csv文件
    cv::VideoWriter *recorder =nullptr; //录制视频的句柄

signals:

};

#endif // IMAGEPROCESSOR_H
