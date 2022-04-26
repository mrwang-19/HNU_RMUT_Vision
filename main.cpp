#include "mainwindow.h"
#include <QApplication>


int main(int argc, char *argv[])
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        exit(status);
    }

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
