#ifndef STEERING_WHEEL_H
#define STEERING_WHEEL_H

#include <QWidget>

class SteeringWheel : public QWidget
{
    Q_OBJECT
public:
    enum StyleType {
        ST_DEFAULT=0,
        ST_ARCBAR
    };
    explicit SteeringWheel(QWidget *parent = nullptr, StyleType type=ST_DEFAULT);
    void setValue(qreal value){
        m_Angle = value;
        update();
    }
    void drawBackGround(QPainter *painter);

signals:

public slots:
protected:
    virtual void paintEvent(QPaintEvent * event);

private:
    qreal m_Angle;
    int m_StyleType;
    QColor m_BorderColor;
};

#endif // STEERING_WHEEL_H
