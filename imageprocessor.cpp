#include <QDateTime>
#include <QDebug>
#include <vector>
#include <fstream>
#include <cmath>

#include "imageprocessor.h"
#define REAL_ENERGY
using namespace cv;
using namespace std;

//初始化识别参数（实际参数会在程序启动时从ui获取）
double ImageProcessor::blueDecay=0.25;
uint8_t ImageProcessor::dilateKernelSize=5;
uint8_t ImageProcessor::binaryThreshold=100;
uint8_t ImageProcessor::maxRadius=200;
uint8_t ImageProcessor::minRadius=110;
uint8_t ImageProcessor::rRadius=20;

ImageProcessor::ImageProcessor(uint16_t height,uint16_t width,uint16_t frameRate,QObject *parent) :
    QObject(parent=nullptr),
    height(height),
    width(width),
    frameRate(frameRate)
{
    qRegisterMetaType<Target>("Target");
}

ImageProcessor::~ImageProcessor()
{
    //如果还在录像则先停止录像
    if(recordingFlag)
        stopRecording();
    processors.waitForDone();
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
    threshold(mid,bin,binaryThreshold,255,THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_ELLIPSE,Point(dilateKernelSize,dilateKernelSize));
    dilate(bin,mid,element);
    Mat kernel = getStructuringElement(MORPH_ELLIPSE,Point(7,7));
    morphologyEx(mid,bin,MORPH_CLOSE,kernel);
    return bin;
}

float distance(Point2f a,Point2f b)
{
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

/**
 * @brief myArctan 求一点距x轴的夹角
 * @param p 点
 * @return 由x轴正方向开始逆时针旋转0～2pi
 */
float myArctan(Point2f p)
{
    float angle = atan2(p.y,p.x);
    return fmod(CV_2PI-angle,CV_2PI);
}

// 2pi/5=1.256637;
// 4p1/5=2.513274;
// 6pi/5=3.769912;
// 8pi/5=5.026548;
/// getJumpAngle 根据跳符前后帧间角度差获取理想的跳转角度
/// \param deltaAngle 跳符前后帧间角度差
/// \return 最接近的理想跳转角度
float getJumpAngle(float deltaAngle)
{
    int minIndex=1;
    float minError=100.0f;
    float delta_angle = fabs(deltaAngle);
    for(int i=1;i<=5;i++)
    {
        float error=fabs(delta_angle-CV_2PI*i/5);
        if(error<minError)
        {
            minError=error;
            minIndex=i;
        }
    }
    if(deltaAngle<0)
        return -CV_2PI*minIndex/5;
    else
        return CV_2PI*minIndex/5;
}

/**
 * @brief ImageProcessor::detect 由预处理过的视频识别能量机关的中心和目标装甲板的像素坐标
 * @param timestamp 从相机得到的纳秒级时间戳
 */
void ImageProcessor::detectTarget(uint64_t timestamp)
{
    static uint64 frame_count=0;
//    static float lastCenterX,lastCenterY;
    Mat original,binaryImage;
    static Target target;
    target.hasTarget = false;
    target.index=frame_count;
    target.timestamp=timestamp;
    frameLock.lock();
    if(frameQueue.size()>1)
    {
        original=frameQueue.takeFirst();
    }
    else
    {
        frameLock.unlock();
        return;
    }
    frameLock.unlock();
    binaryImage=pretreatment(original);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(binaryImage,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
    if(contours.size()>2)
    {
        vector<Point> possibleCenter;
        for(uint i=0;i<contours.size();i++)
        {
            int sub = hierarchy[i][2];
            if(sub!=-1)//有子轮廓
            {
                if(hierarchy[sub][0]==-1)//没有兄弟轮廓
                {
                    auto area = contourArea(contours[sub]);//轮廓面积
                    auto rect = minAreaRect(contours[sub]);//轮廓外接矩形
                    auto mmp=rect.size;
                    float aspectRatio=mmp.height/mmp.width;
                    float areaRatio=area/(rect.size.width*rect.size.height);//面积比，用来衡量轮廓与矩形的相似度
                    if(aspectRatio>1)
                        aspectRatio=1/aspectRatio;
                    //qDebug()<<"面积:"<<area<<",长宽比:"<<aspectRatio<<",面积比:"<<areaRatio<<endl;
                    //TODO:确定实际装甲板面积、长宽比、面积占比
                    if(area>400 && aspectRatio<0.76 && areaRatio>0.75)
                    {
                        target.armorRect=rect;
                        target.hasTarget = true;
                    }
                    //未通过校验
                    else
                    {
                        qDebug()<<"面积:"<<area<<",长宽比:"<<aspectRatio<<",面积比:"<<areaRatio<<endl;
                        goto finish;
                    }
                }
            }
            else
            {
                Point2f center;
                float radius;
                minEnclosingCircle(contours[i],center,radius);
                if(radius<rRadius && radius>8)
                    possibleCenter.push_back(center);
            }
        }
        //如果没有检测到锤子
        if(!target.hasTarget)
        {
            Mat debug=original.clone();
            drawContours(debug,contours,-1,Scalar(0,255,0),2);
            imwrite("/home/rm/图片/NO_"+to_string(target.timestamp)+".bmp",debug);
            goto finish;
        }
        //计算目标装甲板中心的像素坐标
        target.armorCenter=target.armorRect.center;
        Target before;
        //如果上一帧锤子没有及时亮起，就回溯最多5帧
        int back=1;
        int index;
        while(1)
        {
            index=historyTarget.size()-back;
            if(index>0 && !historyTarget[index].hasTarget)
                back++;
            if(index<=0)//第一帧
                goto finish;
            if(historyTarget[index].hasTarget||back>=5)
                break;
        }
        if(back<5)
            before=historyTarget[index];

        ///寻找中心R
        float x2,x3,x4,y2,y3,y4;
        //获取目标装甲板的四个角点
        Point2f vectors[4];
        target.armorRect.points(vectors);
        //计算长宽
        float dd1= distance(vectors[0],vectors[1]);
        float dd2= distance(vectors[1],vectors[2]);
        //筛选出装甲板短边（与锤子柄同向），并计算以目标装甲板中心为原点的短边端点向量
        if(dd1<dd2)
        {
            x2=target.armorCenter.x-vectors[0].x;
            y2=target.armorCenter.y-vectors[0].y;
            x3=target.armorCenter.x-vectors[1].x;
            y3=target.armorCenter.y-vectors[1].y;
            dd2=dd1;
        }
        else
        {
            x2=target.armorCenter.x-vectors[1].x;
            y2=target.armorCenter.y-vectors[1].y;
            x3=target.armorCenter.x-vectors[2].x;
            y3=target.armorCenter.y-vectors[2].y;
            dd1=dd2;
        }
        //计算装甲板短边以目标装甲板中心为原点的向量
        x4=x2-x3;y4=y2-y3;
        //遍历所有可能的中心R点
        for (Point2f p:possibleCenter)
        {
            //计算待选中心和装甲板中心的角度
            float x1=target.armorCenter.x-p.x;
            float y1=target.armorCenter.y-p.y;

            float angle=0;
            float d1 = distance(p,target.armorCenter);

            //计算与目标装甲板中心的夹角
            angle=acos((x4*x1+y1*y4)/dd1/d1)*57.3;

            //根据半径范围和与短边（锤子柄）的角度筛选出中心R
            if(d1>minRadius && d1 < maxRadius && (angle<10||angle>170) )
            {
                target.center=p;
                target.radius=d1;
                break;
            }
        }

        //计算目标装甲板中心相对于能量机关中心的像素坐标
        target.normalizedCenter=Point2f(target.armorCenter.x-target.center.x,target.armorCenter.y-target.center.y);
        //计算目标装甲板中心相对于能量机关中心x轴的夹角
        target.armorAngle=myArctan(target.normalizedCenter);

        if(before.hasTarget)
        {

            float delta=target.armorAngle-before.armorAngle;
            float Delta= fabs(delta);
            //跳符判断
            if(historyTarget.size()>100&&Delta>0.9 && Delta < 6.0)
            {
                lastJumpAngle=getJumpAngle(delta);
                qDebug()<<frame_count<<delta<<lastJumpAngle;
                target.jumpFlag=true;
                indexOfLastJump=historyTarget.size();//本帧角标
                emit energyJumped();
                Mat debug=original.clone();
                circle(debug,target.center,15,Scalar(0,0,255),-1);
                circle(debug,target.armorCenter,15,Scalar(0,255,255),-1);
                drawContours(debug,contours,-1,Scalar(0,255,0),5);
                imwrite("/home/rm/图片/JUMP_"+to_string(target.timestamp)+".bmp",debug);
            }
            //target.lastArmorAngle=target.armorAngle-lastJumpAngle+CV_2PI;
            target.lastArmorAngle= fmod(target.armorAngle-lastJumpAngle+CV_2PI,CV_2PI);
        }
    }
    else
    {
        target.center=Point2f(width/2,height/2);
//        qDebug()<<frame_count<<"has no target";
    }

finish:
    emit newTarget(target);
    if(recordingFlag)
        (*csv_save)<<target.toString();

    historyLock.lock();
    historyTarget.append(target);
    if(historyTarget.size()>HISTORY_LENGTH)
        historyTarget.pop_front();//移除最早的数据
    indexOfLastJump--;
    historyLock.unlock();
    frame_count++;
}
///
/// \brief ImageProcessor::onNewImage
/// \param img_data 图像数据
/// \param height
/// \param width
/// \param timestamp 从相机得到的纳秒级时间戳
void ImageProcessor::onNewImage(char* img_data,int height,int width,uint64_t timestamp)
{
    Mat frame=Mat(height,width,CV_8UC3);
    //逆向拷贝图像数据，此后相机倒放拍摄的照片已被转正，但通道顺序变为RGB（默认为BGR）
    for(int i=0;i<width*height*3;i++)
        frame.data[width*height*3-i-1]=img_data[i];
    if(recordingFlag)
    {
        recoderLock.lock();
        (*recorder)<<frame;
        recoderLock.unlock();
    }
    frameLock.lock();
    frameQueue.append(frame);
    frameLock.unlock();

    //使用线程池来并行处理图像
    QFuture<void> future = QtConcurrent::run(&processors,this,&ImageProcessor::detectTarget,timestamp);

    //删除帧数据
    delete [] img_data;
}
void ImageProcessor::onNewImage(Mat frame)
{
    //使用视频时采用系统时间戳
    QDateTime dateTime = QDateTime::currentDateTime();
    uint64_t mills_timestamp=dateTime.toMSecsSinceEpoch();
    if(recordingFlag)
    {
        recoderLock.lock();
        (*recorder)<<frame;
        recoderLock.unlock();
    }
    cvtColor(frame,frame,COLOR_BGR2RGB);
    frameLock.lock();
    frameQueue.append(frame);
    frameLock.unlock();
    QFuture<void> future = QtConcurrent::run(&processors,this,&ImageProcessor::detectTarget,mills_timestamp*1000000);//毫秒换纳秒
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
    r=to_string(timestamp)+","+to_string(armorAngle)+","+to_string(lastArmorAngle);
    return r;
}
