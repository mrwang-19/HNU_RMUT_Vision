#include "predictor.h"
#include <QVector>

Predictor::Predictor(ImageProcessor * processor,int samples,QObject *parent) :
    QObject(parent),
    samples(samples),
    processor(processor)
{
    tao=(float)processor->tao/150;
    // 配置求解器
    // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = false;   // 输出到cout
    options.num_threads=3;
    timerID=startTimer(30);
}

//非线性拟合代价函数
struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST ( double x, double y,float tao) : _tao(tao),_x ( x ), _y ( y ) {}
    // 残差的计算
    float _tao;
    template <typename T>
    bool operator() (
        const T* const phi,       // 待拟合参数，有1维
        T* residual ) const     // 残差
    {
//        residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x ) + abc[1]*T ( _x ) + abc[2] ); // y-exp(ax^2+bx+c)
        residual[0] = T(_y) - (0.8333333334*sin(0.942*_tao)* ceres::sin(1.884*T ( _x )+0.942*_tao+phi[0])+1.305*_tao);
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
    if(processor->historyTarget.size()>samples)
    {
        problem=new ceres::Problem();
        QVector<Target> list(processor->historyTarget);
        int startIndex=list.size()-1-samples;
        Target start=list[startIndex];
        startTimestamp=start.timestamp;
        int count=0;
        for(int i=startIndex;i<list.size();i++)
        {
            if(list[i].angleDifference>0.1)
            {
                count++;
                problem->AddResidualBlock (     // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                    new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 1> (
                        new CURVE_FITTING_COST ((double)(list[i].timestamp-start.timestamp)/1000,(double)list[i].angleDifference,tao)
                    ),
                        nullptr,            // 核函数，这里不使用，为空
                        _phi                 // 待估计参数
                );
            }
        }
        if(count>samples-10)
        {
            problem->SetParameterLowerBound(_phi,0,-3.1415926536);
            problem->SetParameterUpperBound(_phi,0,3.1415926536);
            ceres::Solver::Summary summary;                // 优化信息
            ceres::Solve ( options,problem, &summary );  // 开始优化
//            cout<<summary.BriefReport() <<endl;
            phi=_phi[0];
            emit newTao(startTimestamp/1000.0,_phi[0]);
        }
        delete problem;
    }
}
Point2f Predictor::predictPoint(float predictTime)
{
    Target currentTarget=processor->historyTarget.last();
    Point2f tmp;
    float timePassed=(QDateTime::currentDateTime().toMSecsSinceEpoch()-startTimestamp)/1000.0;
//    float predictAngleDifference=0.8333333334*sin(0.942*predictTime)* ceres::sin(1.884*(currentTarget.timestamp-startTimestamp)/1000+0.942*predictTime+phi)+1.305*predictTime;
    float predictAngleDifference=0.8333333333*sin(0.942*predictTime)*sin(1.884*(timePassed)+0.942*predictTime+phi)+1.305*predictTime;
    float x=currentTarget.normalizedCenter.x;
    float y=currentTarget.normalizedCenter.y;
    //顺时针旋转
    if(!processor->rotateDirection)
        predictAngleDifference=-predictAngleDifference;
    tmp.x=x*cos(predictAngleDifference)-y*sin(predictAngleDifference);
    tmp.y=y*cos(predictAngleDifference)+x*sin(predictAngleDifference);
    return currentTarget.center+tmp;
}
