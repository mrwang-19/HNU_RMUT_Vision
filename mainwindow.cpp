#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFutureInterfaceBase>
#include <QDateTime>
#include <QDebug>
#include <vector>

//202-4-24
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
    pointer_=this;
}

MainWindow::~MainWindow()
{
    if(hDevice!=nullptr)
    {
        on_OpenButton_clicked();
        while (!processorHandler.isFinished()) {
            ;
        }
    }
    delete ui;
}
/// 采集完成回调函数，每拍一帧就会在大恒SDK的采集线程中被调用一次
/// \brief MainWindow::OnFrameCallbackFun
/// \param pFrame 包含了刚采集的一帧的数据
///
void GX_STDC MainWindow::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
    if (pFrame->status == GX_FRAME_STATUS_SUCCESS)
    {
        char *pRGB24Buf = new char[pFrame->nWidth * pFrame->nHeight * 3]; //输 出 图 像 RGB 数 据
        if (pRGB24Buf == NULL)
        {
            return;
        }
            else
        {
            memset(pRGB24Buf,0,pFrame->nWidth * pFrame->nHeight * 3 * sizeof( char));
            //缓 冲 区 初 始 化
        }
        DX_BAYER_CONVERT_TYPE cvtype = RAW2RGB_NEIGHBOUR3; //选 择 插 值 算 法
        DX_PIXEL_COLOR_FILTER nBayerType = DX_PIXEL_COLOR_FILTER(BAYERBG);
        //选 择 图 像 Bayer 格 式
        bool bFlip = false;
        VxInt32 DxStatus = DxRaw8toRGB24(pFrame->pImgBuf,pRGB24Buf,pFrame->nWidth,pFrame->nHeight,cvtype,nBayerType,bFlip);
        if (DxStatus != DX_OK)
        {
            if (pRGB24Buf != NULL)
            {
                delete []pRGB24Buf;
                pRGB24Buf = NULL;
             }
            return;
        }
        //发射信号，传输图片数据
        emit pointer_->newImage(pRGB24Buf,pFrame->nHeight,pFrame->nWidth);
    }
    return;
}

void MainWindow::on_OpenButton_clicked()
{
    if(hDevice==nullptr)
    {
        GX_OPEN_PARAM stOpenParam;
        exposureTime=ui->exposureSpinBox->value();
        width=ui->widthSpinBox->value();
        height=ui->heightSpinBox->value();
//        QString SN=ui->SNEdit->text();
//        char *sn = new char[SN.length()];
//        strcpy(sn,SN.toStdString().c_str());
//        //    stOpenParam.openMode = GX_OPEN_SN;
//        ////    stOpenParam.pszContent = sn;
//        //    stOpenParam.pszContent = "KE0210020155";
        uint32_t nDeviceNum=0;
        auto status = GXUpdateDeviceList(&nDeviceNum, 1000);
        if (status == GX_STATUS_SUCCESS&&nDeviceNum> 0)
        {
            stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
            stOpenParam.openMode = GX_OPEN_INDEX;
            const char * tmp="1";
            stOpenParam.pszContent = (char*)tmp;
            auto ststus=GXOpenDevice(&stOpenParam, &hDevice);
            if(ststus==GX_STATUS_SUCCESS)
            {
                //初始化相机参数
                cam_init();
                //注册采集回调函数
                GXRegisterCaptureCallback(hDevice, NULL,OnFrameCallbackFun);
                //发送开采命令
                GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
                //创建处理线程
                processor=new ImageProcessor(height,width,1000000/exposureTime,ui->blueDecaySpinBox->value());
                processor->moveToThread(&processorHandler);
                connect(this,&MainWindow::newImage,processor,&ImageProcessor::onNewImage);
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
                predictor = new Predictor(processor,200);
                connect(&predictorHandler,&QThread::finished,predictor,&Predictor::deleteLater);
                predictor->moveToThread(&predictorHandler);
                predictorHandler.start();
//                initDraw();
                timerID=startTimer(33);
                ui->OpenButton->setText("关闭");
            }
        }
    }
    else
    {
        killTimer(timerID);
        //发送停采命令
        GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
        //关闭相机
        GXCloseDevice(hDevice);
        //删除相机句柄
        hDevice=nullptr;
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
 * @param hDevice 相机句柄
 */
bool MainWindow::cam_init()
{
    //自动白平衡
    auto status = GXSetEnum(hDevice,GX_ENUM_BALANCE_WHITE_AUTO,GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    //固定曝光时长
    status &= GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    //设置曝光时长
    status &= GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposureTime);
    //设置分辨率
    status &= GXSetInt(hDevice, GX_INT_WIDTH, width);
    status &= GXSetInt(hDevice, GX_INT_HEIGHT, height);
    //设置偏移量，确保画面中心为相机中心
    status &= GXSetInt(hDevice, GX_INT_OFFSET_X, (1280-width)/2);
    status &= GXSetInt(hDevice, GX_INT_OFFSET_Y, (1024-height)/2);
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

            if(processor->frameQueue.size()>0)
            {
                auto p=predictor->predictPoint(1.5);
                Mat img=processor->frameQueue.last();
                circle(img,tmp.center,15,Scalar(0,255,0),-1);
                circle(img,tmp.armorCenter,15,Scalar(255,0,0),-1);
                circle(img,p,15,Scalar(255,255,0),-1);
                QImage ori=QImage((const uchar*)img.data,width,height,QImage::Format_RGB888);
//                QImage prc=QImage((const uchar*)processor->binaryImage->data,width,height,QImage::Format_Indexed8);
                ui->OriginalImage->setPixmap(QPixmap::fromImage(ori));
//                ui->ProcessedImage->setPixmap(QPixmap::fromImage(prc));
            }
            if(tmp.hasTarget)
            {

//                qDebug()<<"main:"<<tmp.index<<" "<<timestamp;
                ui->angleLable->setNum(tmp.armorAngle);
                ui->centerLable->setText(QString::number(tmp.center.x,'f',4)+","+QString::number(tmp.center.y,'f',4));
                ui->armorLable->setText(QString::number(tmp.armorCenter.x,'f',4)+","+QString::number(tmp.armorCenter.y,'f',4));
            }
            ui->pitchAngleLable->setText(tr("%1").arg(transceiver->recvFrame.pitchAngleGet));
            ui->yawAngleLable->setText(tr("%1").arg(transceiver->recvFrame.yawAngleGet));
            if(ui->checkBoxFollowCenter->isChecked())
            {
                transceiver->sendFrame.yawAngleSet=pid_yaw.pid_calc(tmp.center.x,width/2);
                transceiver->sendFrame.pitchAngleSet=pid_pit.pid_calc(tmp.center.y,height/2);
            }
            else if(ui->checkBoxFollowArmor->isChecked())
            {
                transceiver->sendFrame.yawAngleSet=pid_yaw.pid_calc(tmp.armorCenter.x,width/2+ui->hBaisSpinBox->value());
                transceiver->sendFrame.pitchAngleSet=pid_pit.pid_calc(tmp.armorCenter.y,height/2+ui->vBaisSpinBox->value());
            }
            else
            {
                transceiver->sendFrame.yawAngleSet=0;
                transceiver->sendFrame.pitchAngleSet=0;
            }
        }
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
