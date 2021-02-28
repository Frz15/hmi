#ifndef MAPWIDGET_H
#define MAPWIDGET_H
#include<QGraphicsView>
#include<QLabel>
#include<QMouseEvent>

class MapWidget : public QGraphicsView
{
    Q_OBJECT
public:
    MapWidget(QGraphicsView *parent = nullptr);
 
    void readMap();//读取地图信息
    QPointF mapToMap(QPointF);//用于实时场景坐标系与地图坐标之间的映射，以获得某点的经纬度值
    QPointF latLonToMap(double x, double y);//用于实时场景坐标系与地图坐标之间的映射，以获得某点的经纬度值
    void paintGps(double lon, double lat);

    QGraphicsPixmapItem* pixItem_;
    qreal car_degree_ = 0;

public slots:
    void slotZoom(int);

protected:
    void drawBackground(QPainter *painter, const QRectF &rect);//完成地图显示
//    void mouseMoveEvent(QMouseEvent *event);//鼠标移动

private:
    QPixmap map, car_pic, car_pic_loc;
    qreal zoom = 50;
    QLabel* viewCoord;
    QLabel* sceneCoord;
    QLabel* mapCoord;
    double x1,y1;
    double x2,y2;
};
 
#endif // MAPWIDGET_H
