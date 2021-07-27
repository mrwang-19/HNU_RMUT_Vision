#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QTime>

#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "Daheng_inc/DxImageProc.h"
#include "Daheng_inc/GxIAPI.h"

#include "imageprocessor.h"
#include "chartpainter.h"
#include "anglesolver.h"
#include "transceiver.h"
#include "predictor.h"
#include "camera.h"
#include "pid.h"

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

    void on_blueDecaySpinBox_valueChanged(double arg1);

    void on_checkBoxFollowPredict_stateChanged(int arg1);

    void on_checkBoxFollowCurrent_stateChanged(int arg1);

    void on_yawKpSpinBox_valueChanged(double arg1);

    void on_yawKiSpinBox_valueChanged(double arg1);

    void on_yawKdSpinBox_valueChanged(double arg1);

    void on_pitKpSpinBox_valueChanged(double arg1);

    void on_pitKdSpinBox_valueChanged(double arg1);

    void on_pitKiSpinBox_valueChanged(double arg1);

    void on_shootButton_clicked();

    void on_thresholdspinBox_valueChanged(int arg1);

    void on_dilateKernelSizeSpinBox_valueChanged(int arg1);

    void on_maxRadiusSpinBox_valueChanged(int arg1);

    void on_minRadiusSpinBox_valueChanged(int arg1);

    void on_rRadiusSpinBox_valueChanged(int arg1);

signals:
    void startRecording(QString path);
    void stopRecording();

protected:
    void timerEvent(QTimerEvent *e);

private:
    //变量
    Ui::MainWindow *ui;
    Camera cam;                             //相机包装类
    ImageProcessor * processor = nullptr;   //图像处理线程类
    QThread processorHandler;               //图像处理线程句柄
    Transceiver * transceiver = nullptr;    //串口收发线程
    QThread transceiverHandler;             //串口收发线程句柄
    Predictor * predictor = nullptr;        //预测线程
    QThread predictorHandler;               //预测线程句柄
    QThread chartPainterHandler;            //绘图线程句柄
    pid pid_yaw=pid(0.015,0.002,0.0,30);    //yaw轴pid类
    pid pid_pit=pid(0.015,0.002,0.0,20);    //pitch轴pid类
    uint64 lastTimestamp;                   //上次迭代时间
    int timerID;                            //定时器ID
    int width,height,exposureTime;          //图像宽度、高度、曝光时长
    float frameRate=0.0f;                   //帧率
    AngleSolver angleSolver;                //角度解算类
    QTime shootTimer;                       //计算射击时间使用
    float tmp_yaw=0.0f,tmp_pit=0.0f;        //针孔模型解算得到的未加补偿时的临时云台角度

    //函数
    bool cam_init();    //初始化相机参数
};
#endif // MAINWINDOW_H
