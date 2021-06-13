#include <QDateTime>
#include <QDebug>
#include <vector>
#include <fstream>
#include <cmath>

#include "imageprocessor.h"

using namespace cv;
using namespace std;

ImageProcessor::ImageProcessor(uint16_t height,uint16_t width,uint16_t frameRate,double blueDecay,QObject *parent) :
    QObject(parent=nullptr),
    height(height),
    width(width),
    frameRate(frameRate),
    blueDecay(blueDecay)
{
    qRegisterMetaType<Target>("Target");
//    orignalImage = new  Mat(height,width,CV_8UC3);
//    binaryImage = new Mat(height,width,CV_8UC1);
}
ImageProcessor::~ImageProcessor()
{
    processors.waitForDone();
//    delete orignalImage;
//    delete binaryImage;
}
/**
 * @brief pretreatment 预处理采集到的图像帧
 * @param frame 采集到的原始图像帧
 * @return 预处理后的二值化图像
 */
Mat ImageProcessor::pretreatment(Mat frame)
{
    Mat channels[3],mid,bin;
    split(frame,channels);
    //衰减蓝色通道
    for(int i=0;i<width*height;i++)
    {
        channels[2].data[i]*=(1-blueDecay);
    }
    //红通道-蓝通道
    subtract(channels[0],channels[2],mid);
    threshold(mid,bin,100,255,THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_ELLIPSE,Point(5,5));
    dilate(bin,mid,element);
    Mat kernel = getStructuringElement(MORPH_ELLIPSE,Point(7,7));
    morphologyEx(mid,bin,MORPH_CLOSE,kernel);
    return bin;
}
/**
 * @brief myArctan 求一点距x轴的夹角
 * @param p 点
 * @return 由x轴正方向开始逆时针旋转0～2pi
 */
float myArctan(Point2f p)
{
    float angle = atan2(p.y,p.x);
    if (p.y < 0.0)
    {
        angle = (CV_2PI + angle);
    }
    return CV_2PI-angle;
}

/**
 * @brief MainWindow::detect 由预处理过的视频识别能量机关的中心和目标装甲板的像素坐标
 * @param src 经过预处理（二值化）的图片
 * @param center 能量机关的中心
 * @param armor 目标装甲板的中心
 * @return 是否检测到目标
 */
void ImageProcessor::detectTarget(uint64_t timestamp)
{
    static uint64 frame_count=0;
    Mat binaryImage;
    Target target;
    target.index=frame_count;
    target.timestamp=timestamp;
    frameLock.lock();
    if(frameQueue.size()>1)
        binaryImage=pretreatment(frameQueue.takeFirst());
    else
    {
        frameLock.unlock();
        return;
    }
    frameLock.unlock();
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    //打印时间戳
//    QDateTime dateTime = QDateTime::fromMSecsSinceEpoch(timestamp);
//    qDebug()<<"processor:"<<frame_count<<" "<<dateTime.toString("hh:mm:ss.zzz");
    findContours(binaryImage,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    //    drawContours(fliped,contours,max,Scalar(0,255,0),5);
    //    drawContours(fliped,contours,min,Scalar(255,0,0),5);
    float max_area=0.0,min_area=10000.0;
    if(contours.size()<2)
    {
        target.center=Point2f(width/2,height/2);
        target.hasTarget=false;
//        qDebug()<<frame_count<<"has no target";
    }
    else
    {
        for(int i=0;i<(int)contours.size();i++)
        {
            auto rect=minAreaRect(contours[i]);
            auto tmp = rect.size.area();
//            qDebug()<<tmp;
            if(tmp>max_area)
            {
                max_area=tmp;
                target.armorRect=rect;
            }
            if(tmp<min_area)
            {
                min_area=tmp;
                target.center=rect.center;
            }
            /*
            qDebug()<<i<<":"<<rect.size.aspectRatio();
            if(abs(rect.size.aspectRatio()-0.6)<0.5)
            {
                max_1=i;
            }
            else {
                max_2=i;
            }
            if()
            minEnclosingCircle(contours[i],center,radius);
            float r=contourArea(contours[i])/(rect.area());
            if(r>rate_1)
            {
                rate_1=r;
                max_1=i;
            }
            r=contourArea(contours[i])/(PI*radius*radius);
            if(r>rate_2)
            {
                rate_1=r;
                max_2=i;
            }*/
        }
        //计算目标装甲板中心的像素坐标
        target.armorCenter=target.armorRect.center;
        //计算目标装甲板中心相对于能量机关中心的像素坐标
        target.normalizedCenter=Point2f(target.armorCenter.x-target.center.x,target.armorCenter.y-target.center.y);
        //计算目标装甲板中心相对于能量机关中心x轴的夹角
        target.armorAngle=myArctan(target.normalizedCenter);
        //计算角度差
        int index=historyTarget.size()+1-tao;
        if(index>=0)
        {
            Target before=historyTarget[index];
            if(before.hasTarget)
            {
                //顺时针旋转
                if(rotateDirection)
                {
                    //如果前tao帧的角度比当前帧大
                    if(before.armorAngle>target.armorAngle)
                    {
                        //角度差即为当前角度减去前tao帧的角度
                        target.angleDifference=before.armorAngle-target.armorAngle;
                    }
                    //如果前tao帧的角度比当前帧小（旋转过程中经过了x轴正半轴）
                    else
                    {
                        target.angleDifference=CV_2PI-target.armorAngle+before.armorAngle;
                    }
                }
                //逆时针旋转
                else
                {
                    //如果前tao帧的角度比当前帧小
                    if(before.armorAngle<target.armorAngle)
                    {
                        target.angleDifference=target.armorAngle-before.armorAngle;
//                        qDebug()<<before.armorAngle<<" "<<target.armorAngle<<" "<<target.angleDifference;
                    }
                    //如果前tao帧的角度比当前帧大（旋转过程中经过了x轴正半轴）
                    else
                    {
                        //角度差即为当前角度减去前tao帧的角度
                        target.angleDifference=CV_2PI+target.armorAngle-before.armorAngle;
                    }
                }
            }
        }
    }
    emit newTarget(target);
    if(recordingFlag)
        (*csv_save)<<target.toString();

    historyLock.lock();
    historyTarget.append(target);
    //TO—DO 确定历史长度
    if(historyTarget.size()>500)
        historyTarget.pop_front();//移除最早的数据
    historyLock.unlock();
//    qDebug()<<"历史目标数："<<historyTarget.size();
    frame_count++;
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
    // 字符串格式化
//    QString timestamp = dateTime.toString("hh:mm:ss.zzz");
//    qDebug()<<QThread::currentThread()<<timestamp;
    uint64_t mills_timestamp=dateTime.toMSecsSinceEpoch();
    Mat frame=Mat(height,width,CV_8UC3);
    //逆向拷贝图像数据，此后相机倒放拍摄的照片已被转正，但通道顺序变为RGB（默认为BGR）
    for(int i=0;i<width*height*3;i++)
        frame.data[width*height*3-i-1]=img_data[i];
    if(recordingFlag)
        (*recorder)<<frame;
    frameLock.lock();
    frameQueue.append(frame);
    frameLock.unlock();
    QFuture<void> future = QtConcurrent::run(&processors,this,&ImageProcessor::detectTarget,mills_timestamp);

//    pretreatment(orignalImage);
//    Target target=detectTarget(mills_timestamp);

    //删除帧数据
    delete [] img_data;
}
void ImageProcessor::onNewImage(Mat frame)
{
    //打印时间戳
    QDateTime dateTime = QDateTime::currentDateTime();
    // 字符串格式化
//    QString timestamp = dateTime.toString("hh:mm:ss.zzz");
//    qDebug()<<QThread::currentThread()<<timestamp;
    uint64_t mills_timestamp=dateTime.toMSecsSinceEpoch();
    cvtColor(frame,frame,COLOR_BGR2RGB);
    frameLock.lock();
    frameQueue.append(frame);
    frameLock.unlock();
    QFuture<void> future = QtConcurrent::run(&processors,this,&ImageProcessor::detectTarget,mills_timestamp);
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
    csv_save=new std::ofstream(path2.toStdString(),std::ios::out);
    recorder=new VideoWriter;
    bool tmp = recorder->open(path.toStdString(),VideoWriter::fourcc('X','V','I','D'),frameRate,Size(width,height),true);
    if(tmp)
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

String Target::toString()
{
    String r;
    r=to_string(timestamp)+","+to_string(armorAngle)+","+to_string(angleDifference);
    return r;
}
