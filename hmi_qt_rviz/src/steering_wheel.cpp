#include "../include/cyrobot_monitor/steering_wheel.h"
#include <QPainter>
#include <QDebug>
#include <qmath.h>
#include <QTime>

SteeringWheel::SteeringWheel(QWidget *parent, StyleType type) :
    QWidget(parent),
    m_StyleType(type)
{
    m_BorderColor = QColor(60,60,60);
    m_Angle=0;  //510
    update();
}

void SteeringWheel::drawBackGround(QPainter *painter)
{
    painter->save();
    painter->setPen(m_BorderColor);
    painter->setBrush(m_BorderColor);

    QString pathName = ":/images/wheel.png";
    QImage sourceImage;
    sourceImage.load(pathName);

//    painter->setRenderHint( QPainter::Antialiasing );

    //将坐标系原点移动到图像中心点
    painter->translate( sourceImage.width()/2.0+40, sourceImage.height()/2.0 +20);
    //正数顺时针旋转，范围-360～360
    painter->rotate(m_Angle);
    painter->drawImage(QPoint(-sourceImage.width()/2.0, -sourceImage.height()/2.0), sourceImage);
    painter->restore();
}

void SteeringWheel::paintEvent(QPaintEvent * event)
{
    QPainter p(this);
//    p.setRenderHints(QPainter::Antialiasing|QPainter::TextAntialiasing);
    drawBackGround(&p);
}
