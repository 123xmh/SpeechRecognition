#ifndef MYLABEL_H
#define MYLABEL_H

#include <QWidget>
#include <QLabel>
class MyLabel : public QLabel
{
    Q_OBJECT
public:
    explicit MyLabel(QWidget *parent = nullptr);
    void paintEvent(QPaintEvent *event);

    void UpdateUpdownAndRollAndRotate(double updown, double roll, double zhizhen);

signals:
private:
    //俯仰角度
    double _updown = 0.0f;
    //横滚角度
    double _roll = 0.0f;
    //旋转角度
    double _rotate = 0;
    //指针角度
    double _zhiZhen = 0.0f;
};

#endif // MYLABEL_H
