#ifndef CHARTPAINTER_H
#define CHARTPAINTER_H

#include <QObject>

class ChartPainter : public QObject
{
    Q_OBJECT
public:
    explicit ChartPainter(QObject *parent = nullptr);

signals:

};

#endif // CHARTPAINTER_H
