#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "Daheng_inc/DxImageProc.h"
#include "Daheng_inc/GxIAPI.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    static MainWindow * pointer_;
    ~MainWindow();

private slots:
    void on_OpenButton_clicked();
    void update_img(char* img_data,int height,int width);
    void on_RecordButton_clicked();

signals:
    void new_image(char* img_data,int height,int width);
private:
    Ui::MainWindow *ui;
    GX_DEV_HANDLE hDevice = nullptr;
    cv::VideoWriter *rec =nullptr;
    bool recording_flag=false;
    static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);
};
#endif // MAINWINDOW_H
