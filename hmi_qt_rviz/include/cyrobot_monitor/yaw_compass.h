#ifndef YAW_COMPASS_H
#define YAW_COMPASS_H

/**
 * 指南针仪表盘控件
 * 1:可设置当前度数
 * 2:可设置精确度
 * 3:可设置是否启用动画及步长
 * 4:可设置边框渐变颜色
 * 5:可设置背景渐变颜色
 * 6:可设置加深和明亮颜色
 * 7:可设置指南指北指针颜色
 * 8:可设置中心点渐变颜色
 */

#include <QWidget>
#include <QPainter>
#include <QDebug>
#include <qmath.h>

class YawCompass : public QWidget
{
    Q_OBJECT
public:
    explicit YawCompass(QWidget *parent = nullptr);
    void setValue(double value){
        currentValue = value;
        update();
    }

signals:
public slots:
protected:
    virtual void paintEvent(QPaintEvent *);
    void drawCrownCircle(QPainter *painter);
    void drawBgCircle(QPainter *painter);
    void drawScale(QPainter *painter);
    void drawScaleNum(QPainter *painter);
    void drawCoverOuterCircle(QPainter *painter);
    void drawCoverInnerCircle(QPainter *painter);
    void drawCoverCenterCircle(QPainter *painter);
    void drawPointer(QPainter *painter);
    void drawCenterCircle(QPainter *painter);
    void drawValue(QPainter *painter);

private:
    double value;                   //目标值
    int precision;                  //精确度,小数点后几位
    bool animation;                 //是否启用动画显示
    double animationStep;           //动画显示时步长
    QColor crownColorStart;         //外边框渐变开始颜色
    QColor crownColorEnd;           //外边框渐变结束颜色
    QColor bgColorStart;            //背景渐变开始颜色
    QColor bgColorEnd;              //背景渐变结束颜色
    QColor darkColor;               //加深颜色
    QColor lightColor;              //明亮颜色
    QColor foreground;              //前景色
    QColor textColor;               //文字颜色
    QColor northPointerColor;       //北方指针颜色
    QColor southPointerColor;       //南方指针颜色
    QColor centerColorStart;        //中心圆渐变开始颜色
    QColor centerColorEnd;          //中心圆渐变结束颜色
    bool reverse;                   //是否倒退
    double currentValue;            //当前值
};

#endif // GAUGECOMPASS_H
