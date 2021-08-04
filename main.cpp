#include "mainwindow.h"
#include <QApplication>
#include <stdlib.h>

int main(int argc, char *argv[])
{
    //TODO:使用前务必授予用户sudo的无密码权限
    system("sudo chmod 777 /dev/tty*");
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        exit(status);
    }

    QApplication a(argc, argv);
    MainWindow w;
    //TODO:注释掉下句代码禁用自启动
    w.start();
    w.show();
    w.showFullScreen();
    return a.exec();
}
