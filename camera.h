#ifndef CAMERA_H
#define CAMERA_H

#include <QObject>
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "Daheng_inc/DxImageProc.h"
#include "Daheng_inc/GxIAPI.h"

//大恒相机或录制视频的包装类
using namespace cv;
using namespace std;
Q_DECLARE_METATYPE(Mat)

class Camera : public QObject {
    Q_OBJECT
public:
    Camera(QObject *parent=nullptr);
    ~Camera();
    bool open();
    bool open(String videoPath);
    bool close();
    bool isOpened();
    bool startCapture();
    bool stopCapture();
    bool setImgSize(uint16_t width,uint16_t height);
    bool setExposureTime(uint32_t exposureTime);
    bool setExposureMode(uint8_t exposureMode);
    bool setWhiteBalanceMode(uint8_t whiteBalanceMode);
    uint16_t getHeight();
    uint16_t getWidth();

private:
    static Camera * pointer_;               //为在静态函数中发射信号而创建的指针
    GX_DEV_HANDLE hDevice = nullptr;        //相机句柄
    int timerID;                            //定时器ID
    VideoCapture * videoFile=nullptr;
    double frameRate;

    /**
     * @brief OnFrameCallbackFun 采集完成回调函数，每拍一帧就会在大恒SDK的采集线程中被调用一次
     * @param pFrame 包含了刚采集的一帧的数据
     */
    static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);

protected:
    void timerEvent(QTimerEvent *e);

signals:
    //使用真实相机时发送原始图像数据
    void newImage(char* img_data,int height,int width);
    //使用录制视频时发送Mat对象
    void newImage(Mat img);
};

#endif // CAMERA_H
