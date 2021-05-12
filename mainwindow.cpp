#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFutureInterfaceBase>
#include <QDateTime>
#include <QDebug>
#include <vector>

//202-4-24
using namespace cv;
using namespace std;
//Mat src(1024,1280,CV_8UC3);
#define WIDTH 640
#define HEIGHT 480
#define PI 3.1415926
#define _150_FPS

MainWindow* MainWindow::pointer_=nullptr;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->ImagelLabel->setScaledContents(true);
    rec=new VideoWriter;
    pointer_=this;
    connect(this,&MainWindow::new_image,this,&MainWindow::update_img,Qt::QueuedConnection);
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
        //发射信号，传输图片数据
        emit pointer_->new_image(pRGB24Buf,pFrame->nHeight,pFrame->nWidth);
    }
//    QDateTime dateTime = QDateTime::currentDateTime();
//    QString timestamp = dateTime.toString("yyyy-MM-dd hh:mm:ss.zzz");
//    qDebug()<<timestamp;
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
            //初始化相机参数
            cam_init(hDevice);
            //注册采集回调函数
            GXRegisterCaptureCallback(hDevice, NULL,OnFrameCallbackFun);
            //发送开采命令
            GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
        }
    }
}
Mat pretreatment(Mat frame)
{
    Mat channels[3],mid,binary;
    split(frame,channels);
    subtract(channels[2],channels[1],mid);
    //cv::imshow("img2",channels[1]);
    threshold(mid,binary,100,255,THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_ELLIPSE,Point(5,5));
    dilate(binary,mid,element);
    Mat kernel = getStructuringElement(MORPH_ELLIPSE,Point(7,7));
    morphologyEx(mid,binary,MORPH_CLOSE,kernel);
    return binary;
//    cvThreshold(mid,binary,100,255,CV_THRESH_BINARY);
}
void MainWindow::update_img(char* img_data,int height,int width)
{
    QDateTime dateTime = QDateTime::currentDateTime();
    QString timestamp = dateTime.toString("mm:ss.zzz");
    qDebug()<<timestamp;

    static int fream_count=0;
    auto src = Mat(height,width,CV_8UC3);
    auto fliped = Mat(height,width,CV_8UC3);

    memcpy(src.data,img_data,width*height*3);
//    cv::imshow("img",src);
    cv::flip(src,fliped,-1);
    Mat bin=pretreatment(fliped);
    Point2f center,armor;
    detect(bin,center,armor);
//    drawContours(fliped,contours,max,Scalar(0,255,0),5);
//    drawContours(fliped,contours,min,Scalar(255,0,0),5);
    if(recording_flag)
    {
//        fream_count++;
        (*rec)<<fliped;
        fprintf(csv_file,"%s,%f,%f,%f,%f\n",timestamp.toStdString().c_str(),center.x,center.y,armor.x,armor.y);
//        if(fream_count%5==0)
//        {
//            cvtColor(fliped,src, cv::COLOR_RGB2BGR);
//            QImage img=QImage((const uchar*)src.data,width,height,QImage::Format_RGB888);
//        //    QImage img=QImage((const uchar*)bin.data,width,height,QImage::Format_Indexed8);
//            ui->ImagelLabel->setPixmap(QPixmap::fromImage(img));
//            fream_count=0;
//        }
    }
    else
    {
    circle(fliped,center,20,Scalar(0,255,0),-1);
    circle(fliped,armor,20,Scalar(255,0,0),-1);
//    QImage img=cvMat2QImage(fliped);
    cvtColor(fliped,src, cv::COLOR_RGB2BGR);
    QImage img=QImage((const uchar*)src.data,width,height,QImage::Format_RGB888);
//    QImage img=QImage((const uchar*)bin.data,width,height,QImage::Format_Indexed8);
    ui->ImagelLabel->setPixmap(QPixmap::fromImage(img));
    }
//    cv::imshow("img",fliped);
    //打印时间戳
    delete [] img_data;
}


void MainWindow::on_RecordButton_clicked()
{
    if(!recording_flag)
    {
        QDateTime dateTime = QDateTime::currentDateTime();
        QString timestamp = dateTime.toString("yyyy-MM-dd hh:mm");
        QString path="/home/rm/视频/"+timestamp+".avi";
        QString path2="/home/rm/视频/"+timestamp+".csv";
        char *p=new char[path2.length()];
        strcpy(p,path2.toStdString().data());
        csv_file=fopen(p,"w");
//        qDebug()<<p;
#ifdef _150_FPS
        bool tmp = rec->open(path.toStdString(),CV_FOURCC('X','V','I','D'),150,Size(WIDTH,HEIGHT),true);
#elif defined (_100_FPS)
        bool tmp = rec->open(path.toStdString(),CV_FOURCC('X','V','I','D'),100,Size(WIDTH,HEIGHT),true);
#endif
//        qDebug()<<tmp;
        if(!tmp)
            exit(-2);
        ui->RecordButton->setText("Stop Record");
        recording_flag=true;
    }
    else
    {
        recording_flag=false;
        ui->RecordButton->setText("Record");
        rec->release();
        fflush(csv_file);
        fclose(csv_file);
    }
}

/**
 * @brief MainWindow::cam_init 初始化相机参数，在开采前调用（但本函数不开始采集）
 * @param hDevice 相机句柄
 */
bool MainWindow::cam_init(GX_DEV_HANDLE hDevice)
{
    //自动白平衡
    auto status = GXSetEnum(hDevice,GX_ENUM_BALANCE_WHITE_AUTO,GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    //固定曝光时长
    status &= GXSetEnum(hDevice, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
    //按照帧率设置曝光时长
#ifdef _100_FPS
    status &= GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, 10000.0000);
#elif defined (_150_FPS)
    status &= GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, 6600.0000);
#endif
    //设置分辨率
    status &= GXSetInt(hDevice, GX_INT_WIDTH, WIDTH);
    status &= GXSetInt(hDevice, GX_INT_HEIGHT, HEIGHT);
    //设置偏移量，确保画面中心为相机中心
    status &= GXSetInt(hDevice, GX_INT_OFFSET_X, (1280-WIDTH)/2);
    status &= GXSetInt(hDevice, GX_INT_OFFSET_Y, (1024-HEIGHT)/2);
    return status;
}
/**
 * @brief MainWindow::detect 由预处理过的视频识别能量机关的中心和目标装甲板的像素坐标
 * @param src 经过预处理（二值化）的图片
 * @param center 能量机关的中心
 * @param armor 目标装甲板的中心
 * @return 是否检测到目标
 */
bool MainWindow::detect(cv::Mat src,cv::Point2f &center,cv::Point2f &armor)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(src,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
//    int max=0,min=0;
    float max_area=0.0,min_area=10000.0;
    if(contours.size()<2)
        return false;
    for(int i=0;i<contours.size();i++)
    {
        auto rect=minAreaRect(contours[i]);
        auto tmp = rect.size.area();
        qDebug()<<tmp;
        if(tmp>max_area)
        {
            max_area=tmp;
//            max=i;
            armor=rect.center;
        }
        if(tmp<min_area)
        {
            min_area=tmp;
//            min=i;
            center=rect.center;
        }
//        qDebug()<<i<<":"<<rect.size.aspectRatio();
//        if(abs(rect.size.aspectRatio()-0.6)<0.5)
//        {
//            max_1=i;
//        }
//        else {
//            max_2=i;
//        }
//        if()
//        minEnclosingCircle(contours[i],center,radius);
//        float r=contourArea(contours[i])/(rect.area());
//        if(r>rate_1)
//        {
//            rate_1=r;
//            max_1=i;
//        }
//        r=contourArea(contours[i])/(PI*radius*radius);
//        if(r>rate_2)
//        {
//            rate_1=r;
//            max_2=i;
//        }
    }
    return true;
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
