#include "../include/cyrobot_monitor/yaw_compass.h"

YawCompass::YawCompass(QWidget *parent) :
    QWidget(parent)
{
    value=90;                   //目标值
    precision=0;                  //精确度,小数点后几位
    animation=false;                 //是否启用动画显示
    animationStep=1;           //动画显示时步长
//    crownColorStart=QColor(125,35,39);         //外边框渐变开始颜色
//    crownColorEnd=QColor(224,76,76);           //外边框渐变结束颜色
    crownColorStart=QColor(60,60,60);         //外边框渐变开始颜色
    crownColorEnd=QColor(60,60,60);           //外边框渐变结束颜色
//    bgColorStart=QColor(107,132,160);            //背景渐变开始颜色
//    bgColorEnd=QColor(51,60,72);              //背景渐变结束颜色
    bgColorStart=QColor(160,160,160);            //背景渐变开始颜色
    bgColorEnd=QColor(160,160,160);              //背景渐变结束颜色
    darkColor=QColor(51,60,72);               //加深颜色
    lightColor=QColor(107,132,160);              //明亮颜色
    foreground=QColor(254,254,254);              //前景色
    textColor=QColor(254,254,254);               //文字颜色
    northPointerColor=QColor(199,101,107);       //北方指针颜色
    southPointerColor=QColor(96,161,217);       //南方指针颜色
    centerColorStart=QColor(51,60,72);        //中心圆渐变开始颜色
    centerColorEnd=QColor(107,132,160);          //中心圆渐变结束颜色
    reverse=true;                   //是否倒退
    currentValue=0;            //当前值
    update();
}

void YawCompass::paintEvent(QPaintEvent *)
{
    int width = this->width();
    int height = this->height();
    int side = qMin(width, height);

    //绘制准备工作,启用反锯齿,平移坐标轴中心,等比例缩放
    QPainter painter(this);
    painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
    painter.translate(width / 2, height / 2);
    painter.scale(side / 200.0, side / 200.0);

    //绘制外边框圆
    drawCrownCircle(&painter);
    //绘制背景圆
    drawBgCircle(&painter);
    //绘制刻度
    drawScale(&painter);
    //绘制东南西北标识
    drawScaleNum(&painter);
    //绘制覆盖圆外圆
    drawCoverOuterCircle(&painter);
    //绘制覆盖圆内圆
    drawCoverInnerCircle(&painter);
    //绘制覆盖圆中心圆
    drawCoverCenterCircle(&painter);
    //绘制南北指针
    drawPointer(&painter);
    //绘制中心圆
    drawCenterCircle(&painter);
    //绘制当前值
    drawValue(&painter);
}

void YawCompass::drawCrownCircle(QPainter *painter)
{
    int radius = 86;
    painter->save();
    painter->setPen(Qt::NoPen);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, crownColorStart);
    lineGradient.setColorAt(1, crownColorEnd);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void YawCompass::drawBgCircle(QPainter *painter)
{
    int radius = 75;
    painter->save();
    painter->setPen(Qt::NoPen);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, bgColorStart);
    lineGradient.setColorAt(1, bgColorEnd);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void YawCompass::drawScale(QPainter *painter)
{
    int radius = 70;
    painter->save();

    //总共8格,4格为NESW字母,4格为线条
    int steps = 8;
    double angleStep = 360.0 / steps;

    QPen pen;
    pen.setColor(foreground);
    pen.setCapStyle(Qt::RoundCap);
    pen.setWidth(4);
    painter->setPen(pen);

    //%2整数部分绘制NESW字母,其余绘制线条刻度
    for (int i = 0; i <= steps; i++) {
        if (i % 2 != 0) {
            painter->drawLine(0, radius - 10, 0, radius);
        }

        painter->rotate(angleStep);
    }

    painter->restore();
}

void YawCompass::drawScaleNum(QPainter *painter)
{
    int radius = 73;
    painter->save();
    painter->setPen(foreground);

    QFont font;
    font.setPixelSize(15);
    font.setBold(true);
    painter->setFont(font);

    QRect textRect = QRect(-radius, -radius, radius * 2, radius * 2);
    painter->drawText(textRect, Qt::AlignTop | Qt::AlignHCenter, "N");
    painter->drawText(textRect, Qt::AlignBottom | Qt::AlignHCenter, "S");

    radius -= 2;
    textRect = QRect(-radius, -radius, radius * 2, radius * 2);
    painter->drawText(textRect, Qt::AlignLeft | Qt::AlignVCenter, "W");

    radius -= 2;
    textRect = QRect(-radius, -radius, radius * 2, radius * 2);
    painter->drawText(textRect, Qt::AlignRight | Qt::AlignVCenter, "E");

    painter->restore();
}

void YawCompass::drawCoverOuterCircle(QPainter *painter)
{
    int radius = 53;
    painter->save();
    painter->setPen(Qt::NoPen);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, lightColor);
    lineGradient.setColorAt(1, darkColor);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void YawCompass::drawCoverInnerCircle(QPainter *painter)
{
    int radius = 45;
    painter->save();
    painter->setPen(Qt::NoPen);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, darkColor);
    lineGradient.setColorAt(1, lightColor);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void YawCompass::drawCoverCenterCircle(QPainter *painter)
{
    int radius = 15;
    painter->save();
    painter->setPen(Qt::NoPen);
    painter->setOpacity(0.8);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, lightColor);
    lineGradient.setColorAt(1, darkColor);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void YawCompass::drawPointer(QPainter *painter)
{
    int radius = 60;

    QPolygon pts;

    painter->save();
    painter->setOpacity(0.7);
    painter->setPen(Qt::NoPen);
    painter->setBrush(northPointerColor);
    pts.setPoints(3, -10, 0, 10, 0, 0, radius);
    painter->rotate(currentValue + 180);
    painter->drawConvexPolygon(pts);
    painter->restore();

    painter->save();
    painter->setOpacity(0.7);
    painter->setPen(Qt::NoPen);
    painter->setBrush(southPointerColor);
    pts.setPoints(3, -10, 0, 10, 0, 0, radius);
    painter->rotate(currentValue);
    painter->drawConvexPolygon(pts);
    painter->restore();
}

void YawCompass::drawCenterCircle(QPainter *painter)
{
    int radius = 12;
    painter->save();
    painter->setOpacity(1.0);
    painter->setPen(Qt::NoPen);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, centerColorStart);
    lineGradient.setColorAt(1, centerColorEnd);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void YawCompass::drawValue(QPainter *painter)
{
    int radius = 85;
    painter->save();
    painter->setPen(textColor);

    QFont font;
    font.setPixelSize(11);
    font.setBold(true);
    painter->setFont(font);
    QRectF textRect(-radius, -radius, radius * 2, radius * 2);
    QString strValue = QString::number(currentValue, 'f', 0);
    painter->drawText(textRect, Qt::AlignCenter, strValue);
    painter->restore();
}
