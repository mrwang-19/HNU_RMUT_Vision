#include "mainwindow.h"
#include <QApplication>
#include <stdlib.h>

int main(int argc, char *argv[])
{
    system("sudo chmod 777 /dev/tty*");
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        exit(status);
    }

    QApplication a(argc, argv);
    MainWindow w;
    //w.start();
    w.show();
    w.showFullScreen();
    return a.exec();
}
