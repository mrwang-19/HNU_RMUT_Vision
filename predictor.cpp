#include "predictor.h"
#include <QVector>
#include <QDebug>

Predictor::Predictor(ImageProcessor * processor,int samples,QObject *parent) :
    QObject(parent),
    samples(samples),
    processor(processor)
{
    tao=(float)processor->tao/processor->frameRate;
    // 配置求解器
    options.linear_solver_type = ceres::DENSE_QR;   // 增量方程如何求解
    options.minimizer_progress_to_stdout = false;   // 不输出到控制台
    options.num_threads=3;
    //启动定时器
    timerID=startTimer(30ms);
}

//非线性拟合代价函数
struct CURVE_FITTING_COST
{
    //构造函数
    CURVE_FITTING_COST ( double x, double y,float tau) : _tau(tau),_x ( x ), _y ( y ) {}
    // 残差的计算
    float _tau;
    template <typename T>
    bool operator() (
        const T* const phi,       // 待拟合参数，有1维
        T* residual ) const     // 残差
    {
//        residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x ) + abc[1]*T ( _x ) + abc[2] ); // y-exp(ax^2+bx+c)
        auto value = 1.305*_tau + 0.41666666667*(-ceres::cos(phi[0]+(1.884*_x)) + ceres::cos(phi[0]+1.884*(-_tau+_x)));
//        auto value = 1.305*_tau+0.4166666667*(-2)* sin(-1.884*_tau/2)*ceres::sin(phi[0]+1.884*_x+(1.884*_tau/2));
        residual[0] = T(_y) - value;
        return true;
    }
    const double _x, _y;    // x,y数据
};

Predictor::~Predictor()
{
    killTimer(timerID);
}
void Predictor::timerEvent(QTimerEvent*)
{
    double _phi[1]={0.0};
    if(processor->historyTarget.size()>(samples+processor->tao))
    {
        problem=new ceres::Problem();
        QVector<Target> list(processor->historyTarget);
        int startIndex=list.size()-1-samples;
//        qDebug()<<"startIndex:"<<startIndex;
        Target currentTarget=list.last();
        // seve the start time
        startTimestamp=currentTarget.timestamp;
        int count=0;
        for(int i=startIndex;i<startIndex+samples;i++)
        {
//            cout<<list[i].angleDifference<<",";
            if(list[i].angleDifference>0.1)
            {
                count++;
                double tmp=((double)(currentTarget.timestamp-list[i].timestamp))/-1000000000;//纳秒换秒
//                qDebug()<<tmp<<","<<(double)list[i].angleDifference;
                problem->AddResidualBlock (     // 向问题中添加误差项
                        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 1> (
                                //以开始拟合的一帧作为时间轴原点，其余帧与其计算相对时间
                                new CURVE_FITTING_COST (tmp,(double)list[i].angleDifference,tao)
                        ),
                        nullptr,            // 核函数，这里不使用，为空
                        _phi                 // 待估计参数
                );
            }
        }
//        cout<<endl;
//            qDebug()<<"count:"<<count;
        //有效样本足够多才能进行拟合
        if(count>samples-10)
        {
            //设定参数边界
//            problem->SetParameterLowerBound(_phi,0,0);
//            problem->SetParameterUpperBound(_phi,0,CV_2PI);
            ceres::Solver::Summary summary;                // 优化信息
            ceres::Solve ( options,problem, &summary );  // 开始优化
//            cout<<summary.BriefReport() <<endl;
            //如果没有跳变则融合上次预测结果与实际时间
//            if(_phi[0]-last_phi<CV_PI)
//                phi=_phi[0]*0.6+(last_phi+0.0159235668789809)*0.4;
//            else
                phi=_phi[0];
            last_phi=phi;   //更新last_phi
//            qDebug()<<phi;
            emit newPhi(startTimestamp, fmod(_phi[0]+CV_2PI,CV_2PI));
        }
        delete problem;
    }
}

float pos_fun(float t,float phi)
{
    return (1.305 * t) - (0.416666666666667 * cos(1.884 *t + phi ));
}

Point2f Predictor::predictPoint(float predictTime)
{
    static float lastX,lastY;
    Point2f tmp;
    float predictAngleDifference=0.0;
//    Target currentTarget;
    currentTarget=processor->historyTarget.last();
    float timePassed=((float)(currentTarget.timestamp-startTimestamp))/1000.0;
    predictAngleDifference= pos_fun(predictTime,phi)-pos_fun(0,phi);
//    qDebug()<<phi<<","<<timePassed+predictTime<<","<<predictAngleDifference;
//    auto angle= fmod(currentTarget.armorAngle+predictAngleDifference+CV_2PI,CV_2PI);
//    emit newPhi(currentTarget.timestamp+(int)(predictTime*1000),angle);
//    float predictAngleDifference=0.8333333333*sin(0.942*predictTime)*sin(1.884*(timePassed)+0.942/predictTime+phi)+1.305*predictTime;
    float x=currentTarget.normalizedCenter.x;
    float y=currentTarget.normalizedCenter.y;
    //顺时针旋转
    if(!processor->rotateDirection)
        predictAngleDifference=-predictAngleDifference;
    tmp.x=x*cos(predictAngleDifference)-y*sin(predictAngleDifference);
//    if(abs(tmp.x-lastX)<10)
    tmp.x=0.1*tmp.x+0.9*lastX;
    lastX=tmp.x;
    tmp.y=y*cos(predictAngleDifference)+x*sin(predictAngleDifference);
//    if(abs(tmp.y-lastY)<10)
    tmp.y=0.1*tmp.y+0.9*lastY;
    lastY=tmp.y;
    return currentTarget.center+tmp;
}
float Predictor::getSpeed(float predictTime)
{
    float speed=0.785*sin(1.884*predictTime+phi)+1.305;
    emit newSpeed(currentTarget.timestamp,speed);
    return speed;
}