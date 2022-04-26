#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFutureInterfaceBase>
#include <QDateTime>
#include <QDebug>

using namespace cv;
//Mat src(1024,1280,CV_8UC3);
MainWindow* MainWindow::pointer_=nullptr;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    pointer_=this;
//    this->self=this;
    connect(this,&MainWindow::new_image,this,&MainWindow::update_img);
}

MainWindow::~MainWindow()
{
    delete ui;
}

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
        emit pointer_->new_image(pRGB24Buf,pFrame->nHeight,pFrame->nWidth);

//        src.resize(pFrame->nHeight,pFrame->nWidth);
//        memcpy(src.data,pRGB24Buf,pFrame->nWidth*pFrame->nHeight*3);

//        imshow("image",src);
    }
    return;
}

void MainWindow::on_OpenButton_clicked()
{
    GX_OPEN_PARAM stOpenParam;
    QString SN=ui->SNEdit->toPlainText();
    char *sn = new char[SN.length()];
    strcpy(sn,SN.toStdString().c_str());
//    stOpenParam.openMode = GX_OPEN_SN;
////    stOpenParam.pszContent = sn;
//    stOpenParam.pszContent = "KE0210020155";
    uint32_t nDeviceNum=0;
    auto status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if (status == GX_STATUS_SUCCESS&&nDeviceNum> 0)
    {
        stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
        stOpenParam.openMode = GX_OPEN_INDEX;
        stOpenParam.pszContent = "1";
        auto ststus=GXOpenDevice(&stOpenParam, &hDevice);
        if(ststus==GX_STATUS_SUCCESS)
        {
            GXRegisterCaptureCallback(hDevice, NULL,OnFrameCallbackFun);
            GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
//            namedWindow("image", WINDOW_NORMAL);
//            while (1) {
//                imshow("img",src);
//            }
        }
    }
}
void MainWindow::update_img(char* img_data,int height,int width)
{
    auto src = Mat(height,width,CV_8UC3);
    memcpy(src.data,img_data,width*height*3);
    cv::imshow("img",src);
//    //output timetamp
    QDateTime dateTime = QDateTime::currentDateTime();
    QString timestamp = dateTime.toString("yyyy-MM-dd hh:mm:ss.zzz");
    qDebug()<<timestamp;
    delete [] img_data;
}
