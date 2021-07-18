#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFutureInterfaceBase>
#include <QDateTime>
#include <QDebug>
#include <vector>

using namespace cv;
using namespace std;

MainWindow* MainWindow::pointer_=nullptr;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->OriginalImage->setScaledContents(true);
    pid_pit.kp=ui->pitKpSpinBox->value();
    pid_pit.ki=ui->pitKiSpinBox->value();
    pid_pit.kd=ui->pitKdSpinBox->value();
    pid_yaw.kp=ui->yawKpSpinBox->value();
    pid_yaw.ki=ui->yawKiSpinBox->value();
    pid_yaw.kd=ui->yawKdSpinBox->value();
    angleSolver.setCameraParam("/home/rm/HNU_RMUT_Version/camera_params.xml", 1);
    shootTimer=QTime::currentTime();
    pointer_=this;
    this->move(100,100);
}

MainWindow::~MainWindow()
{
    if(cam.isOpened())
    {
        on_OpenButton_clicked();
        while (!processorHandler.isFinished()) {
            ;
        }
    }
    delete ui;
}


void MainWindow::on_OpenButton_clicked()
{
    if(!cam.isOpened())
    {
        if(!ui->checkBoxUseFile->isChecked())
        {
            if(cam.open())
            {
                //获取设置参数
                exposureTime=ui->exposureSpinBox->value();
                width=ui->widthSpinBox->value();
                height=ui->heightSpinBox->value();
                //初始化相机参数
                cam_init();
            }
            else
                return;
            //创建处理线程
            processor=new ImageProcessor(height,width,ui->frameRateSpinBox->value(),ui->blueDecaySpinBox->value());
            processor->moveToThread(&processorHandler);
            connect(&cam,static_cast<void (Camera::*)(char*,int,int,uint64_t)>(&Camera::newImage),processor,static_cast<void (ImageProcessor::*)(char *,int,int,uint64_t)>(&ImageProcessor::onNewImage));
            //开采
            cam.startCapture();
        }
        else
        {
            if(!cam.open(ui->videoPathEdit->text().toStdString()))
                return;
            height=cam.getHeight();
            width=cam.getWidth();
            //创建处理线程
            processor=new ImageProcessor(height,width,(uint16_t)cam.getFrameRate(),ui->blueDecaySpinBox->value());
            processor->moveToThread(&processorHandler);
            connect(&cam,static_cast<void (Camera::*)(Mat)>(&Camera::newImage),processor,static_cast<void (ImageProcessor::*)(Mat)>(&ImageProcessor::onNewImage));
            //开采
            cam.startCapture();
        }

        connect(this,&MainWindow::startRecording,processor,&ImageProcessor::startRecording);
        connect(this,&MainWindow::stopRecording,processor,&ImageProcessor::stopRecording);
        connect(&processorHandler,&QThread::finished,processor,&ImageProcessor::deleteLater);
        processorHandler.start();
        //创建收发线程
        transceiver=new Transceiver(ui->serialNameEdit->text());
        transceiver->moveToThread(&transceiverHandler);
        connect(&transceiverHandler,&QThread::finished,transceiver,&ImageProcessor::deleteLater);
        transceiverHandler.start();
        //创建绘图线程
        ui->chartPainter->moveToThread(&chartPainterHandler);
        connect(processor,&ImageProcessor::newTarget,ui->chartPainter,&ChartPainter::onTarget);
        chartPainterHandler.start();
        //创建预测线程
        predictor = new Predictor(processor,150);
        connect(&predictorHandler,&QThread::finished,predictor,&Predictor::deleteLater);
        connect(predictor,&Predictor::newPhi,ui->chartPainter,&ChartPainter::onPhi);
        connect(predictor,&Predictor::newSpeed,ui->chartPainter,&ChartPainter::onSpeed);
        predictor->moveToThread(&predictorHandler);
        predictorHandler.start();
        //启动定时器
        timerID=startTimer(33ms);
        ui->OpenButton->setText("关闭");
    }
    else
    {
        killTimer(timerID);
        //
        cam.close();
        //销毁绘图线程
        chartPainterHandler.quit();
        //销毁预测线程
        predictorHandler.quit();
        //销毁收发线程
        transceiverHandler.quit();
        transceiver=nullptr;
        //销毁处理线程
        processorHandler.quit();
        processor=nullptr;
        //改变按钮文本
        ui->OpenButton->setText("打开");
    }
}

/**
 * @brief MainWindow::timerEvent    主窗口定时器处理函数，按照固定频率给出预测、解算云台角度并刷新主界面上显示的图像
 * @param e
 */
void MainWindow::timerEvent(QTimerEvent*)
{
    static bool flag=true;
    if(processor!=nullptr && transceiver!=nullptr)
    {
        if(processor->historyTarget.size()>0)
        {
            Target tmp=processor->historyTarget.last();

            Mat img;    //最新原始图像的拷贝，用于调试
            if(processor->frameQueue.size()>0)
                processor->frameQueue.last().copyTo(img);

            //刷新目标信息
            if(tmp.hasTarget)
            {
                //预测
                Point2f p;
                float timePassed=shootTimer.elapsed()/1000.0f;
//                qDebug()<<timePassed;
                float predictTime=ui->predictTimeSpinBox->value();
                if(timePassed>predictTime)
                {
                    shootTimer.restart();
                    flag=true;
                }
//                auto lead=0.3*predictor->getSpeed(predictTime-timePassed);
                float lead=ui->leadTimeSpinBox->value();
                if((timePassed>(predictTime-lead))&&flag)
                {
                    transceiver->sendFrame.shootCommand=1;
                    flag=false;
                    //打印时间戳
                    qDebug()<<QThread::currentThread()<<shootTimer.currentTime();
                }
                p=predictor->predictPoint(predictTime-timePassed);
//                p=predictor->predictPoint(0.1);

                //跟随

                //跟随预测点
                if(ui->checkBoxFollowPredict->isChecked())
                {
                    angleSolver.getAngle(p,tmp_yaw,tmp_pit);
                    //PID闭环并加入偏置补偿
                    transceiver->sendFrame.yawAngleSet=pid_yaw.pid_calc(tmp_yaw,ui->hBaisSpinBox->value());
                    transceiver->sendFrame.pitchAngleSet=-pid_pit.pid_calc(tmp_pit,ui->vBaisSpinBox->value());
                }
                //跟随当前点
                else if(ui->checkBoxFollowCurrent->isChecked())
                {
                    angleSolver.getAngle(tmp.armorCenter,tmp_yaw,tmp_pit);
                    //PID闭环并加入偏置补偿
                    transceiver->sendFrame.yawAngleSet=pid_yaw.pid_calc(tmp_yaw,ui->hBaisSpinBox->value());
                    transceiver->sendFrame.pitchAngleSet=-pid_pit.pid_calc(tmp_pit,ui->vBaisSpinBox->value());
                }
                else
                {
                    transceiver->sendFrame.yawAngleSet=0;
                    transceiver->sendFrame.pitchAngleSet=0;
                }
                //标出关键点
                circle(img,tmp.center,15,Scalar(0,255,0),-1);
                circle(img,tmp.armorCenter,15,Scalar(255,0,0),-1);
                circle(img,p,15,Scalar(255,255,0),-1);
                ui->angleLable->setNum(tmp.armorAngle);
                ui->centerLable->setText(QString::number(tmp.center.x,'f',4)+","+QString::number(tmp.center.y,'f',4));
                ui->armorLable->setText(QString::number(tmp.armorCenter.x,'f',4)+","+QString::number(tmp.armorCenter.y,'f',4));
            }

            //更新ui
            QImage ori=QImage((const uchar*)img.data,width,height,QImage::Format_RGB888);
            ui->OriginalImage->setPixmap(QPixmap::fromImage(ori));
            //刷新云台角度
            ui->pitchAngleLable->setText(tr("%1").arg(transceiver->recvFrame.pitchAngleGet));
            ui->yawAngleLable->setText(tr("%1").arg(transceiver->recvFrame.yawAngleGet));

            ui->calcPitLable->setText(QString::number(tmp_pit));
            ui->calcYawLable->setText(QString::number(tmp_yaw));

        }
        else
        {
            transceiver->sendFrame.yawAngleSet=0;
            transceiver->sendFrame.pitchAngleSet=0;
        }
        //重绘图表
        ui->chartPainter->replot();
    }
}
void MainWindow::on_RecordButton_clicked()
{
    if(processor!=nullptr)
    {
        if(processor->recordingFlag)
        {
            ui->RecordButton->setText("停止录制");
            emit stopRecording();
        }
        else
        {
            ui->RecordButton->setText("开始录制");
            emit startRecording(ui->savePathEdit->text());
        }
    }
}

/**
 * @brief MainWindow::cam_init 初始化相机参数，在开采前调用（但本函数不开始采集）
 */
bool MainWindow::cam_init()
{
    bool status=true;
    //连续自动白平衡
    status&=cam.setExposureMode(1);
    //固定曝光时长
    status&=cam.setExposureMode(0);
    //设置曝光时长
    status&=cam.setExposureTime(exposureTime);
    //设置帧率控制
    status&=cam.setFrameRate(ui->frameRateSpinBox->value());
    //设置分辨率
    status&=cam.setImgSize(width,height);
    return status;
}

void MainWindow::on_blueDecaySpinBox_valueChanged(double arg1)
{
    if(processor!=nullptr)
        processor->blueDecay=arg1;
}

void MainWindow::on_checkBoxFollowPredict_stateChanged(int)
{
    if(ui->checkBoxFollowCurrent->isChecked())
        ui->checkBoxFollowCurrent->setCheckState(Qt::Unchecked);
}

void MainWindow::on_checkBoxFollowCurrent_stateChanged(int)
{
    if(ui->checkBoxFollowPredict->isChecked())
        ui->checkBoxFollowPredict->setCheckState(Qt::Unchecked);
}

void MainWindow::on_yawKpSpinBox_valueChanged(double arg1)
{
    pid_yaw.kp=arg1;
}

void MainWindow::on_yawKiSpinBox_valueChanged(double arg1)
{
    pid_yaw.ki=arg1;
}

void MainWindow::on_yawKdSpinBox_valueChanged(double arg1)
{
    pid_yaw.kd=arg1;
}

void MainWindow::on_pitKpSpinBox_valueChanged(double arg1)
{
    pid_pit.kp=arg1;
}

void MainWindow::on_pitKdSpinBox_valueChanged(double arg1)
{
    pid_pit.kd=arg1;
}

void MainWindow::on_pitKiSpinBox_valueChanged(double arg1)
{
    pid_pit.ki=arg1;
}

void MainWindow::on_shootButton_clicked()
{
    transceiver->sendFrame.shootCommand=1;
}
