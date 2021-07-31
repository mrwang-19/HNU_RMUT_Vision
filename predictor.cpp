#include "predictor.h"
#include <QVector>
#include <QDebug>

Predictor::Predictor(ImageProcessor * processor,int samples,QObject *parent) :
    QObject(parent),
    samples(samples),
    processor(processor)
{
    tau=processor->frameRate/2;
    tao=(float)tau/processor->frameRate;
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
    if(processor->historyTarget.size()>(samples+tau))
    {
        problem=new ceres::Problem();
        QVector<Target> list(processor->historyTarget);
        int startIndex=list.size()-1-samples;
        //qDebug()<<"startIndex:"<<startIndex;
        Target currentTarget=list.last();
        //可能的开始时间，因为拟合结果最后不一定被采纳
        uint64_t possibleStartTimestamp=currentTarget.timestamp;
        int count=0;
        for(int i=startIndex;i<startIndex+samples;i++)
        {
            float angleDifference;
            //如果跳变帧包含在区间内，则用跳变后的角度折算至跳变前，计算角度差
            //qDebug()<<"last jump:"<<processor->indexOfLastJump;
            if(list[i-tau].hasTarget&list[i].hasTarget)
            {
                //顺时针旋转
                if(processor->rotateDirection)
                {
                    //如果tau帧内包含着跳符，则使用折算角度
                    if(i-tau < processor->indexOfLastJump && i > processor->indexOfLastJump)
                    {
                        //如果前tao帧的角度比当前帧大
                        if(list[i-tau].armorAngle>list[i].lastArmorAngle)
                        {
                            //角度差即为当前角度减去前tao帧的角度
                            angleDifference=list[i-tau].armorAngle-list[i].lastArmorAngle;
                        }
                            //如果前tao帧的角度比当前帧小（旋转过程中经过了x轴正半轴）
                        else
                        {
                            angleDifference=CV_2PI-list[i].lastArmorAngle+list[i-tau].armorAngle;
                        }
                    }
                    else
                    {
                        if(list[i-tau].armorAngle>list[i].armorAngle)
                        {
                            //角度差即为当前角度减去前tao帧的角度
                            angleDifference=list[i-tau].armorAngle-list[i].armorAngle;
                        }
                            //如果前tao帧的角度比当前帧小（旋转过程中经过了x轴正半轴）
                        else
                        {
                            angleDifference=CV_2PI-list[i].armorAngle+list[i-tau].armorAngle;
                        }
                    }
                }
                //逆时针旋转
                else
                {
                    //如果tau帧内包含着跳符，则使用折算角度
                    if(i-tau < processor->indexOfLastJump && i > processor->indexOfLastJump)
                    {
                        //如果前tao帧的角度比当前帧小,则用当前角度减去前τ帧的角度
                        if(list[i-tau].armorAngle<list[i].lastArmorAngle)
                        {
                            angleDifference=list[i].lastArmorAngle-list[i-tau].armorAngle;
                        }
                            //如果前tao帧的角度比当前帧大（旋转过程中经过了x轴正半轴）,则用前τ帧的角度减去当前角度
                        else
                        {
                            //角度差即为当前角度减去前tao帧的角度
                            angleDifference=CV_2PI+list[i].lastArmorAngle-list[i-tau].armorAngle;
                        }
                    }
                    else
                    {
                        //如果前tao帧的角度比当前帧小,则用当前角度减去前τ帧的角度
                        if(list[i-tau].armorAngle<list[i].armorAngle)
                        {
                            angleDifference=list[i].armorAngle-list[i-tau].armorAngle;
                        }
                            //如果前tao帧的角度比当前帧大（旋转过程中经过了x轴正半轴）,则用前τ帧的角度减去当前角度
                        else
                        {
                            //角度差即为当前角度减去前tao帧的角度
                            angleDifference=CV_2PI+list[i].armorAngle-list[i-tau].armorAngle;
                        }
                    }

                }

//                if(processor->rotateDirection)
//                {
//                    if(i-tau < processor->indexOfLastJump)
//                        angleDifference=list[i-tau].armorAngle-list[i].lastArmorAngle;
//                    else
//                        angleDifference=list[i-tau].armorAngle-list[i].armorAngle;
//                }
//                else
//                {
//                    if(i-tau < processor->indexOfLastJump)
//                        angleDifference=list[i].lastArmorAngle-list[i-tau].armorAngle;
//                    else
//                        angleDifference=list[i].armorAngle-list[i-tau].armorAngle;
//                }
                //qDebug()<<angleDifference;
                //angleDifference = fmod(angleDifference+CV_2PI,CV_2PI);
                emit newPhi(list[i].timestamp,angleDifference);
                //qDebug()<<"angleDifference:"<<angleDifference;
                if(angleDifference>0.1&&angleDifference<1.5)
                {
                    count++;
                    //为了防止精度丢失，要先用后面的时间戳（大）减去前面的时间戳（小）然后才能转换成负数
                    double tmp=((double)(possibleStartTimestamp-list[i].timestamp))/-1000000000.0;//纳秒换秒
                    //qDebug()<<tmp<<","<<angleDifference;
                    problem->AddResidualBlock (     // 向问题中添加误差项
                            // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 1> (
                                    //以开始拟合的一帧作为时间轴原点，其余帧与其计算相对时间
                                    new CURVE_FITTING_COST (tmp,(double)angleDifference,tao)
                            ),
                            nullptr,            // 核函数，这里不使用，为空
                            _phi                 // 待估计参数
                    );
                }
            }
        }
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
            startTimestamp=possibleStartTimestamp;//更新拟合函数原点的实际时间
            emit newPhi(startTimestamp, fmod(_phi[0]+CV_2PI,CV_2PI));
        }
        else
        {
            //qDebug()<<"未能信任拟合结果，有效样本数："<<count;
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
    Target currentTarget;
    currentTarget=processor->historyTarget.last();
    float timePassed=((float)(currentTarget.timestamp-startTimestamp))/1000000000.0;
    predictAngleDifference = pos_fun(predictTime+timePassed,phi)-pos_fun(timePassed,phi);
//    qDebug()<<phi<<timePassed<<predictAngleDifference;
    auto angle= fmod(currentTarget.armorAngle-predictAngleDifference+CV_2PI,CV_2PI);
    emit newSpeed(currentTarget.timestamp+(int)(predictTime*1000000000),angle);
//    float predictAngleDifference=0.8333333333*sin(0.942*predictTime)*sin(1.884*(timePassed)+0.942/predictTime+phi)+1.305*predictTime;
    float x=currentTarget.normalizedCenter.x;
    float y=currentTarget.normalizedCenter.y;
    //顺时针旋转
    //qDebug()<<"direction:"<<processor->rotateDirection;
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
    Target currentTarget;
    currentTarget=processor->historyTarget.last();
    float timePassed=((float)(currentTarget.timestamp-startTimestamp))/1000000000.0;
    float speed=0.785*sin(1.884*(predictTime+timePassed)+phi)+1.305;
    //emit newSpeed(currentTarget.timestamp,speed);
    return speed;
}