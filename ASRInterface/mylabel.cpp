#include "mylabel.h"
#include "cmath"
#include <QPainter>
#include <QDebug>
MyLabel::MyLabel(QWidget *parent) : QLabel(parent)
{
   // setFixedSize(200,200);
}

void MyLabel::paintEvent(QPaintEvent *event)
{
    //painter0:画外侧两个半圆
    //painter1:画内部两个半圆
    //painter2:画刻度和文字
    //painter3:画中间垂直刻度
    //painter4:画指针
    Q_UNUSED(event);
    QPainter painter0(this);
    //旋转角度
    double rotate = 0;
    int centerX = this->width()/2;
    int centerY = this->height()/2;
    painter0.setRenderHint(QPainter::Antialiasing);//抗锯齿
    painter0.translate(centerX,centerY);
    painter0.rotate(rotate);
    //圆的半径
    int radius = qMin(this->width(),this->height())/2-5;
    //画一个整圆
    painter0.setBrush(QBrush(QColor("#0a1b45")));
    QPen pen0("#2a579b");
    pen0.setWidth(3);
    painter0.setPen(pen0);
    painter0.drawEllipse(-radius,-radius,radius*2,radius*2);


    //画两个分开的半圆
/*  QLinearGradient gradient1(centerX,centerY-radius,centerX,centerY+radius);
    gradient1.setColorAt(0,QColor("#003173"));
    gradient1.setColorAt(1,QColor("#054da3"));
    painter0.setBrush(gradient1);
    painter0.drawPie(-radius,-radius,radius*2,radius*2,0*16,180*16);
    QLinearGradient gradient2(centerX,centerY-radius,centerX,centerY+radius);
    gradient2.setColorAt(0,QColor("#037a88"));
    gradient2.setColorAt(1,QColor("#06394e"));
    painter0.setBrush(gradient2);
    painter0.drawPie(-radius,-radius,radius*2,radius*2,180*16,180*16);*/
    //画小刻度
    QPainter painter2(this);
    painter2.translate(centerX,centerY);
    painter2.setRenderHint(QPainter::Antialiasing, true);
    QPen pen2(Qt::white);
    pen2.setWidth(width()/100);
    painter2.setPen(pen2);
    //小刻度长度
    int lenMin = radius/15;
    //小刻度到外圆的距离
    int _tkMin = radius/15;
    for(int i=0;i<24;i++)
    {
        painter2.drawLine(-(radius-_tkMin),0,-(radius-_tkMin-lenMin),0);
        painter2.rotate(15);
    }
    //画大刻度和文字
    //设置刻度线长度
    pen2.setWidth(width()/80);
    painter2.setPen(pen2);
    // 设置字体
    QFont font("Arial", width()*3/55);
    painter2.setFont(font);
    //设置填充颜色
    painter2.setBrush(QBrush(Qt::white));
    int lenMax = radius/8;//刻度线长度
    int tkMax = 0;//刻度线到外圆的距离
    QStringList stringlist;
    stringlist <<"SE"<<" S"<<"SW"<<" W"<<"NW"<<" N"<<"NE"<<" E";
    int index = 0;
    for(int i=0;i<360;i+=45)
    {
         //画刻度
         painter2.drawLine(-(radius-tkMax),0,-(radius-tkMax-lenMax),0);
         painter2.rotate(45);
         //画三角
         if((i+45)%90 == 0)
         {
            QPoint point1(radius-radius/50,0);
            QPoint point2(radius,radius*5/250);
            QPoint point3(radius,-radius*5/250);
            painter2.drawPolygon(QPolygonF()<<point1<<point2<<point3);
         }
         //画文字
         //先把画刻度线的painter2对象保存
         painter2.save();
         //把painter2坐标系移动到写文字的位置
         painter2.translate(radius-lenMax-width()*3/50, 0);
         //旋转坐标系使其水平（画出来的字不歪斜）
         painter2.rotate(-i-45);
         //画文字
         painter2.drawText(-(width()/21),width()/40,stringlist[index++]);
         //把painter2恢复到保存前的状态，继续画刻度
         painter2.restore();

    }

    //画内部圆
    //内部圆与外边框距离
     const int wh = qMin(this->width(),this->height())/2*5/12;
    //绘制圆的半径
    int r = qMin(this->width(),this->height())/2 - wh;
    //俯仰角
    double upDown = _updown;
    //滚动角
    double roll = _roll;
    //翻滚角处理
    //绘制使用的俯仰角
    double realAngle = acos((upDown/90.0f))* 180.0f / 3.1415926 * 2;
    //绘制俯仰角时的旋转角
    double rotateAngle = (180.0f - realAngle)/2;
    // 将角度转换为弧度
    qreal rad = rotateAngle * 3.1415926 /180.0f;
    // 创建一个QTransform对象，用于旋转路径
    QTransform transform;
    transform.translate(r+wh, r+wh);
    transform.rotateRadians(-rad);
    transform.translate(-r-wh, -r-wh);
    //滚动角处理
    //将角度转换为弧度
    qreal rollRad = roll * 3.1415926 /180.0f;
    // 创建一个QTransform对象，用于旋转路径
    QTransform rollTransform;
    rollTransform.translate(r+wh, r+wh);
    rollTransform.rotateRadians(rollRad);
    rollTransform.translate(-r-wh, -r-wh);

    QPainter painter1(this);
    painter1.setRenderHint(QPainter::Antialiasing);//抗锯齿
 /*   //画外圆
    QRectF rectE(0,0,2*r+10,2*r+10);
    painter1.setRenderHint(QPainter::Antialiasing, true);
    painter1.setBrush(Qt::white);
    painter1.drawEllipse(rectE);*/

    //绘制上半圆
    QRectF rectf(wh, wh, 2*r, 2*r);
    QPainterPath path;
    path.moveTo(2*r+wh, r+wh);
    path.arcTo(rectf, 0, realAngle);
    //旋转正
    path = transform.map(path);
    //翻滚角旋转
    path = rollTransform.map(path);
    QPen pen(Qt::white);
    painter1.setPen(pen);
    painter1.setBrush(QBrush(QColor("#00a7ff")));
    painter1.drawPath(path);
    //绘制下半圆
    QPainterPath path2;
    path2.moveTo(2*r+wh, r+wh);
    path2.arcTo(rectf, 0, -360 + realAngle);
    //旋转正
    path2 = transform.map(path2);
    //翻滚角旋转
    path2 = rollTransform.map(path2);
    painter1.setBrush(QBrush(QColor("#e3fffe")));
    painter1.drawPath(path2);
    //画中间垂直刻度
    QPainter painter3(this);
    painter3.translate(centerX,centerY-r);
    QPen pen3(Qt::black);
    pen3.setWidth(2);
    painter3.setPen(pen3);
    QFont font3("CESI黑体-GB13000", width()/25);
    painter3.setFont(font3);
    int size =width()*3/50;
    int longLine = width()/25;
    int shortLine = width()/125;
    QStringList mlist;
    mlist << " " <<" 135" <<" 90" <<" 45" <<" 0" <<"-45" <<"-90" <<"-135" <<" ";
    index = 0;
    for(int i=0;i<41;i++)
    {
        if(i%5 == 0)
        {
            pen3.setWidth(1);
            painter3.setPen(pen3);
            painter3.drawLine(-longLine,(i+1)*2*r/42,longLine,(i+1)*2*r/42);

            pen3.setColor(QColor(Qt::red));
            painter3.setPen(pen3);
            painter3.drawText(size,(i+1)*2*r/42-size/2,size*2,size,Qt::AlignCenter,mlist[index++]);
            pen3.setColor(QColor(Qt::black));
            painter3.setPen(pen3);

        }
        else
        {

            pen3.setWidth(1);
            painter3.setPen(pen3);
            painter3.drawLine(-shortLine,(i+1)*2*r/42,shortLine,(i+1)*2*r/42);

        }
    }

    //画指针
    QPainter painter4(this);
    //旋转角度
    double zhizhen = _zhiZhen;
    painter4.setRenderHint(QPainter::Antialiasing);//抗锯齿
    painter4.translate(width()/2,height()/2);
    painter4.rotate(zhizhen);
    QPen pen4(Qt::red);
    pen4.setWidth(width()/30);
    painter4.setPen(pen4);
    //设置填充颜色
//  painter4.setBrush(QBrush(Qt::white));
    int zhizhen_lenMax = radius/6;//刻度线长度
    int zhizhen_tkMax = 0;//刻度线到外圆的距离
    painter4.drawLine(-(radius-zhizhen_tkMax),0,-(radius-zhizhen_tkMax-zhizhen_lenMax),0);

}

void MyLabel::UpdateUpdownAndRollAndRotate(double updown, double roll,double zhizhen)
{
    _updown = updown;
    _roll = roll;
    _zhiZhen = zhizhen+90;
    update();
}
