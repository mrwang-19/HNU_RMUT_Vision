#include <QDateTime>
#include <QDebug>
#include <vector>
#include <fstream>

#include "imageprocessor.h"
using namespace cv;
using namespace std;
ImageProcessor::ImageProcessor(uint16_t height,uint16_t width,uint16_t frameRate,QObject *parent) :
    QObject(parent=nullptr),
    height(height),
    width(width),
    frameRate(frameRate)
{

}

/**
 * @brief pretreatment 预处理采集到的图像帧
 * @param frame 采集到的原始图像帧
 * @return 预处理后的二值化图像
 */
Mat ImageProcessor::pretreatment(Mat frame)
{
    Mat channels[3],mid,binary;
    split(frame,channels);
    //红通道-蓝通道
    subtract(channels[0],channels[2],mid);
    threshold(mid,binary,100,255,THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_ELLIPSE,Point(5,5));
    dilate(binary,mid,element);
    Mat kernel = getStructuringElement(MORPH_ELLIPSE,Point(7,7));
    morphologyEx(mid,binary,MORPH_CLOSE,kernel);
    return binary;
}
/**
 * @brief MainWindow::detect 由预处理过的视频识别能量机关的中心和目标装甲板的像素坐标
 * @param src 经过预处理（二值化）的图片
 * @param center 能量机关的中心
 * @param armor 目标装甲板的中心
 * @return 是否检测到目标
 */
bool ImageProcessor::detectTarget(cv::Mat src,cv::Point2f &center,cv::Point2f &armor)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(src,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    //    drawContours(fliped,contours,max,Scalar(0,255,0),5);
    //    drawContours(fliped,contours,min,Scalar(255,0,0),5);
//    int max=0,min=0;
    float max_area=0.0,min_area=10000.0;
    if(contours.size()<2)
        return false;
    for(int i=0;i<(int)contours.size();i++)
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
///
/// \brief ImageProcessor::onNewImage
/// \param img_data 图像数据
/// \param height
/// \param width
///
void ImageProcessor::onNewImage(char* img_data,int height,int width)
{
    //打印时间戳
    QDateTime dateTime = QDateTime::currentDateTime();
    QString timestamp = dateTime.toString("mm:ss.zzz");
    qDebug()<<timestamp;

    static int fream_count=0;
    auto src = Mat(height,width,CV_8UC3);
    //逆向拷贝图像数据，此后相机倒放拍摄的照片已被转正，但通道顺序变为RGB（默认为BGR）
    for(int i=0;i<width*height*3;i++)
        src.data[width*height*3-i-1]=img_data[i];

    Mat bin=pretreatment(src);
    Point2f center,armor;
    detectTarget(bin,center,armor);

    if(recordingFlag)
    {
//        fream_count++;
        (*recorder)<<src;
        (*csv_save)<<timestamp.toStdString()<<","<<center.x<<","<<center.y<<","<<armor.x<<","<<armor.y;
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
//    circle(src,center,20,Scalar(0,255,0),-1);
//    circle(src,armor,20,Scalar(255,0,0),-1);
//    QImage ori=QImage((const uchar*)src.data,width,height,QImage::Format_RGB888);
////    QImage prc=QImage((const uchar*)bin.data,width,height,QImage::Format_Indexed8);
//    ui->OriginalImage->setPixmap(QPixmap::fromImage(ori));
//    ui->ProcessedImage->setPixmap(QPixmap::fromImage(prc));
    }
    //删除帧数据
    delete [] img_data;
}
/**
 * @brief ImageProcessor::startRecording    打开录制
 * @param savePath  保存路径（文件夹）
 */
void ImageProcessor::startRecording(QString savePath)
{
    QDateTime dateTime = QDateTime::currentDateTime();
    QString timestamp = dateTime.toString("yyyy-MM-dd hh:mm");
    QString path=savePath+timestamp+".avi";
    QString path2=savePath+timestamp+".csv";
//    char *p=new char[path2.length()];
//    strcpy(p,path2.toStdString().data());
    csv_save=new std::ofstream(path2.toStdString(),std::ios::out);
    recorder=new VideoWriter;
    bool tmp = recorder->open(path.toStdString(),VideoWriter::fourcc('X','V','I','D'),frameRate,Size(width,height),true);
    if(!tmp)
    {
        recordingFlag=true;
    }
}
/**
 * @brief ImageProcessor::stopRecording 关闭录制
 */
void ImageProcessor::stopRecording()
{
    recordingFlag=false;
    if(csv_save!=nullptr)
    {
        csv_save->close();
        delete csv_save;
        csv_save=nullptr;
    }
    if(recorder!=nullptr)
    {
        recorder->release();
        delete recorder;
        recorder=nullptr;
    }
}
