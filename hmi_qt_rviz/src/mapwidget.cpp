#include "../include/cyrobot_monitor/mapwidget.h"
#include <QSlider>
#include <QGridLayout>
#include <QFile>
#include <QTextStream>
#include <QGraphicsScene>
#include <QPushButton>
#include <QGraphicsPixmapItem>
#include <math.h>
#include <QThread>
#include <QDebug>
#include <iostream>

MapWidget::MapWidget(QGraphicsView *parent):QGraphicsView(parent)
{
    readMap();//读取地图信息
 
    zoom=50;
 
    int width=map.width();
    int height=map.height();
    QGraphicsScene* scene=new QGraphicsScene(this);
    scene->setSceneRect(-width/2,-height/2,width,height);
    setScene(scene);
    setCacheMode(CacheBackground);

    //用于地图缩放的滑动条
    QSlider* slider=new QSlider;
    slider->setOrientation(Qt::Vertical);
    slider->setRange(1,100);
    slider->setTickInterval(10);
    slider->setValue(50);
    slider->setStyleSheet("background-color: transparent; border:0px");
    connect(slider,SIGNAL(valueChanged(int)),this,SLOT(slotZoom(int)));
 
    QLabel* zoominLabel=new QLabel;
    zoominLabel->setScaledContents(true);
    zoominLabel->setPixmap(QPixmap(":/images/zoomout.png"));
    zoominLabel->setStyleSheet("background-color: transparent; border:0px");
    QLabel* zoomoutLabel=new QLabel;
    zoomoutLabel->setScaledContents(true);
    zoomoutLabel->setPixmap(QPixmap(":/images/zoomin.png"));
    zoomoutLabel->setStyleSheet("background-color: transparent; border:0px");
    //坐标值显示区
    QLabel* label1=new QLabel(tr("GraphicsView:"));
    viewCoord=new QLabel;
    QLabel* label2=new QLabel(tr("GraphicsScene:"));
    sceneCoord=new QLabel;
    QLabel* label3=new QLabel(tr("GPS:"));
    mapCoord=new QLabel;
    label1->setStyleSheet("font: bold; font-size:17px; color: rgb(0, 255, 0); border:0px;");
    viewCoord->setStyleSheet("font: bold; font-size:17px; color: rgb(0, 255, 0); border:0px;");
    label2->setStyleSheet("font: bold; font-size:17px; color: rgb(0, 255, 0); border:0px;");
    sceneCoord->setStyleSheet("font: bold; font-size:17px; color: rgb(0, 255, 0); border:0px;");
    label3->setStyleSheet("font: bold; font-size:17px; color: rgb(255, 0, 0); border:0px;");
    mapCoord->setStyleSheet("font: bold; font-size:17px; color: rgb(255, 0, 0); border:0px;");
 
    //坐标显示区布局
    QGridLayout* gridLayout=new QGridLayout;
    gridLayout->addWidget(label3,0,0);
    gridLayout->addWidget(mapCoord,0,1);
    gridLayout->setSizeConstraint(QLayout::SetFixedSize);
    QFrame* coordFrame=new QFrame;
    coordFrame->setLayout(gridLayout);
    coordFrame->setStyleSheet("border:0px; background-color: transparent;");

    //缩放控制子局
    QVBoxLayout* zoomLayout=new QVBoxLayout;
    zoomLayout->addWidget(zoominLabel);
    zoomLayout->addWidget(slider);
    zoomLayout->addWidget(zoomoutLabel);
 
    //坐标显示区域布局
    QVBoxLayout* coordLayout=new QVBoxLayout;
    coordLayout->addWidget(coordFrame);
    coordLayout->addStretch();
 
    //主布局
    QHBoxLayout* mainLayout=new QHBoxLayout;
    mainLayout->addLayout(zoomLayout);
    mainLayout->addLayout(coordLayout);
    mainLayout->addStretch();
    mainLayout->setMargin(30);
    mainLayout->setSpacing(10);
    setLayout(mainLayout);

    setWindowTitle("Map Widget");
    setMinimumSize(655,460);
    update();
}

void MapWidget::readMap()
{
    QString mapName;
    QFile mapFile(":/maps.txt");
    if(mapFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        mapFile.seek(0);
        QTextStream ts(&mapFile);
        ts.setRealNumberPrecision(11);
        if(!ts.atEnd())
        {
            QString line = ts.readLine();
            mapName = line;
            line = ts.readLine();
            QStringList list = line.split(" ");
            x1=list[0].toDouble();
            y1=list[1].toDouble();
            x2=list[2].toDouble();
            y2=list[3].toDouble();
//            qDebug()<<fixed<<qSetRealNumberPrecision(14)<<x1<<" "<<y1<<" "<<x2<<" "<<y2;
        }
    } else {
        qDebug()<<"Can't open file.";
    }

    map.load(mapName);
    car_pic.load(":/images/car_1.png");
}

void MapWidget::paintGps(double lon, double lat)
{
    mapCoord->setText(QString::number(lon, 'f', 10)+", "+QString::number(lat, 'f', 10));
    QPointF scenePoint = latLonToMap(lon, lat);
    pixItem_ = new QGraphicsPixmapItem(car_pic);
    //pixItem_ = new QGraphicsPixmapItem(car_pic_loc);
    this -> scene() -> clear();
    pixItem_ -> setTransformOriginPoint(scenePoint.x(), scenePoint.y());
    pixItem_ -> setRotation(car_degree_);
    pixItem_ -> setOffset(scenePoint.x()-car_pic.width()/2, scenePoint.y()-car_pic.height()/2);
    this -> scene() -> addItem(pixItem_);
    this -> centerOn(scenePoint.x(), scenePoint.y());
    update();
}

void MapWidget::slotZoom(int value)
{
    qreal s;
    if(value>zoom)//放大
    {
        s=pow(1.01,(value-zoom));
    }
    else {
        s=pow(1/1.01,(zoom-value));
    }
    scale(s,s);
    zoom=value;
}
void MapWidget::drawBackground(QPainter *painter, const QRectF &rect)
{
    painter->drawPixmap(int(sceneRect().left()),int(sceneRect().top()),map);
}

//void MapWidget::mouseMoveEvent(QMouseEvent *event)
//{
//    //QGraphicsView坐标
//    QPoint viewPoint=event->pos();
//    viewCoord->setText(QString::number(viewPoint.x())+", "+
//                       QString::number(viewPoint.y()));
//    //QGraphicsScene坐标
//    QPointF scenePoint=mapToScene(viewPoint);
//    sceneCoord->setText(QString::number(scenePoint.x())+", "+
//                       QString::number(scenePoint.y()));
//    //地图坐标（经、纬度值）
//    QPointF latLon=mapToMap(scenePoint);
//    mapCoord->setText(
//                QString::number(latLon.x(), 'f', 10)+", "+
//                                       QString::number(latLon.y(), 'f', 10));
//}

QPointF MapWidget::mapToMap(QPointF p)
{
    QPointF latLon;
//    qreal w=sceneRect().width();
//    qreal h=sceneRect().height();
    qreal w=651-205;
    qreal h=1358-280;
//    qreal lon=(y1*10000-((h/2+p.y())*abs(y1*10000-y2*10000)/h))/10000.0;
//    qreal lat=(x1*10000+((w/2+p.x())*abs(x1*10000-x2*10000)/w))/10000.0;
    qreal lon=(y1-((h/2+p.y()-70)*qAbs(y1-y2)/h));
    qreal lat=(x1+((w/2+p.x()+80)*qAbs(x1-x2)/w));
    latLon.setX(lat);
    latLon.setY(lon);
    return latLon;
}

QPointF MapWidget::latLonToMap(double x, double y)
{
//    qDebug()<<fixed<<qSetRealNumberPrecision(13)<<x<<" "<<y;
    qreal w=651-205;
    qreal h=1358-280;
    QPointF scenePoint;
//    double x_scene = ((x - x1) * w) / qAbs(x1*10000 - x2*10000) * 10000 - w / 2;
//    double y_scene = ((y1 - y) * h) / qAbs(y1*10000 - y2*10000) * 10000 - h / 2;
    double x_scene = ((x - x1) * w) / qAbs(x1 - x2) - w / 2;
    double y_scene = ((y1 - y) * h) / qAbs(y1 - y2) - h / 2;
    scenePoint.setX(x_scene-70);
    scenePoint.setY(y_scene+80);
//    qDebug()<<fixed<<qSetRealNumberPrecision(13)<< x_scene <<" "<<y_scene;
    return scenePoint;
}
