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
//        auto value=1.305*_tao+0.41666666667*(-ceres::cos(phi[0]+(1.884*(-_tao+_x)))+ceres::cos(phi[0]+1.884*_x));
        auto value = 1.305*_tau+0.4166666667*(-2)* sin(-1.884*_tau/2)*ceres::sin(phi[0]+1.884*_x+(1.884*_tau/2));
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
        int startIndex=list.size()-1-samples-processor->tao;
//        qDebug()<<"startIndex:"<<startIndex;
        startTarget=list[startIndex];
        // seve the start time
        startTimestamp=startTarget.timestamp;
        int count=0;
        for(int i=startIndex+processor->tao;i<startIndex+processor->tao+samples;i++)
        {
//            cout<<list[i].angleDifference<<",";
//            if(list[i].angleDifference>0.1)
//            {
//                count++;
                double tmp=((double)(list[i].timestamp-list[startIndex+processor->tao].timestamp))/1000;
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
//            }
        }
        cout<<endl;
//            qDebug()<<"count:"<<count;
        //有效样本足够多才能进行拟合
//        if(count>samples-10)
//        {
            //设定参数边界
//            problem->SetParameterLowerBound(_phi,0,0);
//            problem->SetParameterUpperBound(_phi,0,CV_2PI);
            ceres::Solver::Summary summary;                // 优化信息
            ceres::Solve ( options,problem, &summary );  // 开始优化
//            cout<<summary.BriefReport() <<endl;
//            //如果没有跳变则融合上次预测结果与实际时间
//            if(_phi[0]-last_phi<CV_PI)
//                phi=(_phi[0]+last_phi+0.0159235668789809)/2;
//            else
            phi=_phi[0];
            last_phi=phi;   //更新last_phi
//            qDebug()<<phi;
//            emit newPhi(startTimestamp, fmod(_phi[0]+CV_2PI,CV_2PI));
//        }
        delete problem;
    }
}

float pos_fun(float t,float phi)
{
    return (1.305 * t) - (0.416666666666667 * cos(1.884 *t + phi ));
}

Point2f Predictor::predictPoint(float predictTime)
{
    Point2f tmp;
    float predictAngleDifference=0.0;
    Target currentTarget;
    currentTarget=processor->historyTarget.last();
    float timePassed=((float)(currentTarget.timestamp-startTimestamp))/1000.0;
    predictAngleDifference= pos_fun(timePassed+predictTime,phi)-pos_fun(0,phi);
//    qDebug()<<phi<<","<<timePassed+predictTime<<","<<predictAngleDifference;
    auto angle= fmod(startTarget.armorAngle+predictAngleDifference+CV_2PI,CV_2PI);
    emit newPhi(currentTarget.timestamp+(int)(predictTime*1000),angle);
//    float predictAngleDifference=0.8333333333*sin(0.942*predictTime)*sin(1.884*(timePassed)+0.942/predictTime+phi)+1.305*predictTime;
    float x=startTarget.normalizedCenter.x;
    float y=startTarget.normalizedCenter.y;
    //顺时针旋转
    if(!processor->rotateDirection)
        predictAngleDifference=-predictAngleDifference;
    tmp.x=x*cos(predictAngleDifference)-y*sin(predictAngleDifference);
    tmp.y=y*cos(predictAngleDifference)+x*sin(predictAngleDifference);
    return startTarget.center+tmp;
}
