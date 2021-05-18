#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>

#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "Daheng_inc/DxImageProc.h"
#include "Daheng_inc/GxIAPI.h"

#include "imageprocessor.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    static MainWindow * pointer_;
    ~MainWindow();

private slots:
    void on_OpenButton_clicked();
    void on_RecordButton_clicked();

signals:
    void newImage(char* img_data,int height,int width);
    void startRecording(QString path);
    void stopRecording();

protected:
    void timerEvent(QTimerEvent *e);

private:
    //变量
    Ui::MainWindow *ui;
    GX_DEV_HANDLE hDevice = nullptr;        //相机句柄
    ImageProcessor * processor = nullptr;   //图像处理线程类
    QThread processorHandler;               //图像处理线程句柄
//    bool open_flag=false;                   //是否已经开采标记

    int width,height,exposureTime;
    //函数
    bool cam_init();
    static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);
    static QImage cvMat2QImage(const cv::Mat& mat);
};
#endif // MAINWINDOW_H
