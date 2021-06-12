#include "camera.h"

Camera* Camera::pointer_=nullptr;

Camera::Camera(QObject *parent):QObject(parent=nullptr)
{
    //注册Mat类型为MetaType
    qRegisterMetaType<Mat>("CV_Matrix");
    //为工具指针赋值
    pointer_=this;
}

bool Camera::open()
{
    if(hDevice==nullptr&&videoFile==nullptr)
    {
        GX_OPEN_PARAM stOpenParam;
        uint32_t nDeviceNum=0;
        auto status = GXUpdateDeviceList(&nDeviceNum, 1000);
        if (status == GX_STATUS_SUCCESS&&nDeviceNum> 0)
        {
            stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
            stOpenParam.openMode = GX_OPEN_INDEX;
            const char * tmp="1";
            stOpenParam.pszContent = (char*)tmp;
            status=GXOpenDevice(&stOpenParam, &hDevice);
            return status;
        }
    }
    return false;
}

bool Camera::open(String videoPath)
{
    if(hDevice==nullptr&&videoFile==nullptr)
    {
        videoFile=new VideoCapture(videoPath);
        if(!videoFile->isOpened())
            return false;
        frameRate=videoFile->get(CAP_PROP_FPS);
        return true;
    }
    return false;
}

void GX_STDC Camera::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
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

void Camera::timerEvent(QTimerEvent*)
{
    if(videoFile->isOpened())
    {
        Mat frame;
        videoFile->read(frame);
        emit newImage(frame);
    }
}

bool Camera::startCapture()
{
    if(hDevice!=nullptr)
    {
        //注册采集回调
        auto status=GXRegisterCaptureCallback(hDevice, NULL,OnFrameCallbackFun);
        //发送开采命令
        status=GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
        return status;
    }
    else if (videoFile!=nullptr)
    {
        timerID=startTimer(1000/frameRate);
        return true;
    }
    return false;
}

bool Camera::stopCapture()
{
    if(hDevice!=nullptr)
    {
        //发送停采命令
        auto status=GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
        return status;
    }
    else if (videoFile!=nullptr)
    {
        killTimer(timerID);
        return true;
    }
    return false;
}

bool Camera::setImgSize(uint16_t width,uint16_t height)
{
    if(hDevice!=nullptr)
    {
        //设置分辨率
        auto status = GXSetInt(hDevice, GX_INT_WIDTH, width);
        status &= GXSetInt(hDevice, GX_INT_HEIGHT, height);
        //设置偏移量，确保画面中心为相机中心
        status &= GXSetInt(hDevice, GX_INT_OFFSET_X, (1280-width)/2);
        status &= GXSetInt(hDevice, GX_INT_OFFSET_Y, (1024-height)/2);
        return status;
    }
    return false;
}

bool Camera::setExposureTime(uint32_t exposureTime)
{
    if(hDevice!=nullptr)
    {
        //设置曝光时长
        return GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposureTime);
    }
    return false;
}

bool Camera::setExposureMode(uint8_t exposureMode)
{
    if(hDevice!=nullptr)
    {
        switch (exposureMode)
        {
        case 0:
            return GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
            break;
        case 1:
            return GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
            break;
        }
    }
    return false;
}
bool Camera::setWhiteBalanceMode(uint8_t whiteBalanceMode)
{
    if(hDevice!=nullptr)
    {

    }
    return false;
}
