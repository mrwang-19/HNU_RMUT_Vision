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
    ui->ProcessedImage->setScaledContents(true);
    pid_pit.kp=ui->pitKpSpinBox->value();
    pid_pit.ki=ui->pitKiSpinBox->value();
    pid_pit.kd=ui->pitKdSpinBox->value();
    pid_yaw.kp=ui->yawKpSpinBox->value();
    pid_yaw.ki=ui->yawKiSpinBox->value();
    pid_yaw.kd=ui->yawKdSpinBox->value();
    angleSolver.setCameraParam("/home/rm/HNU_RMUT_Version/camera_params.xml", 1);
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
            //开采
            cam.startCapture();
            //创建处理线程
            processor=new ImageProcessor(height,width,1000000/exposureTime,ui->blueDecaySpinBox->value());
            processor->moveToThread(&processorHandler);
            connect(&cam,static_cast<void (Camera::*)(char*,int,int)>(&Camera::newImage),processor,static_cast<void (ImageProcessor::*)(char *,int,int)>(&ImageProcessor::onNewImage));
        }
        else
        {
            if(!cam.open(ui->videoPathEdit->text().toStdString()))
                return;
            height=cam.getHeight();
            width=cam.getWidth();
            //开采
            cam.startCapture();
            //创建处理线程
            processor=new ImageProcessor(height,width,(uint16_t)cam.getFrameRate(),ui->blueDecaySpinBox->value());
            processor->moveToThread(&processorHandler);
            connect(&cam,static_cast<void (Camera::*)(Mat)>(&Camera::newImage),processor,static_cast<void (ImageProcessor::*)(Mat)>(&ImageProcessor::onNewImage));
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
    //设置分辨率
    status&=cam.setImgSize(width,height);
    return status;
}
/**
 * @brief MainWindow::timerEvent    定时器处理函数，用于按照固定频率刷新主界面上显示的图像
 * @param e
 */
void MainWindow::timerEvent(QTimerEvent*)
{
    if(processor!=nullptr && transceiver!=nullptr)
    {
        if(processor->historyTarget.size()>0)
        {
            Target tmp=processor->historyTarget.last();
            Point2f p;
            //预测目标并刷新预览图
            if(processor->frameQueue.size()>0)
            {
                //打印时间戳
                QDateTime dateTime = QDateTime::currentDateTime();
//                // 字符串格式化
//                QString timestamp = dateTime.toString("hh:mm:ss.zzz");
//                qDebug()<<QThread::currentThread()<<timestamp;
                float timePassed=((float)(dateTime.toMSecsSinceEpoch()-lastTimestamp))/1000.0;
                float predictTime=ui->predictTimeSpinBox->value();
                if(timePassed>predictTime)
                {
                    lastTimestamp=dateTime.currentDateTime().toMSecsSinceEpoch();
                }
                if(timePassed>(predictTime-0.1))
                    transceiver->sendFrame.shootCommand=1;
                p=predictor->predictPoint(predictTime-timePassed);
//                p=predictor->predictPoint(0.1);
                Mat img;
                processor->frameQueue.last().copyTo(img);
                circle(img,tmp.center,15,Scalar(0,255,0),-1);
                circle(img,tmp.armorCenter,15,Scalar(255,0,0),-1);
                circle(img,p,15,Scalar(255,255,0),-1);
                QImage ori=QImage((const uchar*)img.data,width,height,QImage::Format_RGB888);
                ui->OriginalImage->setPixmap(QPixmap::fromImage(ori));
            }
            //刷新目标信息
            if(tmp.hasTarget)
            {

//                qDebug()<<"main:"<<tmp.index<<" "<<timestamp;
                ui->angleLable->setNum(tmp.armorAngle);
                ui->centerLable->setText(QString::number(tmp.center.x,'f',4)+","+QString::number(tmp.center.y,'f',4));
                ui->armorLable->setText(QString::number(tmp.armorCenter.x,'f',4)+","+QString::number(tmp.armorCenter.y,'f',4));
            }
            //刷新云台角度
            ui->pitchAngleLable->setText(tr("%1").arg(transceiver->recvFrame.pitchAngleGet));
            ui->yawAngleLable->setText(tr("%1").arg(transceiver->recvFrame.yawAngleGet));
            //PID闭环
            if(ui->checkBoxFollowCenter->isChecked())
            {
                transceiver->sendFrame.yawAngleSet=pid_yaw.pid_calc(p.x,width/2+ui->hBaisSpinBox->value());
                transceiver->sendFrame.pitchAngleSet=pid_pit.pid_calc(p.y,height/2+ui->vBaisSpinBox->value());
            }
            else if(ui->checkBoxFollowArmor->isChecked())
            {
//                transceiver->sendFrame.yawAngleSet=pid_yaw.pid_calc(tmp.armorCenter.x,width/2+ui->hBaisSpinBox->value());
//                transceiver->sendFrame.pitchAngleSet=pid_pit.pid_calc(tmp.armorCenter.y,height/2+ui->vBaisSpinBox->value());
                auto pnt=Point2f(p.x,p.y-ui->vBaisSpinBox->value());
                float y,p;
                angleSolver.getAngle(pnt,y,p);
                ui->calcPitLable->setText(QString::number(p));
                ui->calcYawLable->setText(QString::number(y));
                transceiver->sendFrame.yawAngleSet=transceiver->recvFrame.yawAngleGet-y;
                transceiver->sendFrame.pitchAngleSet=transceiver->recvFrame.pitchAngleGet+p;
            }
            else
            {
                transceiver->sendFrame.yawAngleSet=0;
                transceiver->sendFrame.pitchAngleSet=0;
            }
        }
        //重绘图表
        ui->chartPainter->replot();
    }
}

QImage MainWindow::cvMat2QImage(const cv::Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if(mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for(int row = 0; row < mat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if(mat.type() == CV_8UC4)
    {
        qDebug() << "CV_8UC4";
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}

void MainWindow::on_blueDecaySpinBox_valueChanged(double arg1)
{
    if(processor!=nullptr)
        processor->blueDecay=arg1;
}

void MainWindow::on_checkBoxFollowCenter_stateChanged(int)
{
    if(ui->checkBoxFollowArmor->isChecked())
        ui->checkBoxFollowArmor->setCheckState(Qt::Unchecked);
}

void MainWindow::on_checkBoxFollowArmor_stateChanged(int)
{
    if(ui->checkBoxFollowCenter->isChecked())
        ui->checkBoxFollowCenter->setCheckState(Qt::Unchecked);
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
