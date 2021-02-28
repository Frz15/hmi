/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <QSlider>
#include <QGridLayout>
#include <QFile>
#include <QTextStream>
#include <QGraphicsScene>
#include <math.h>
#include <QGraphicsView>
#include <QLabel>
#include <QMouseEvent>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "../include/cyrobot_monitor/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	ui.menubar->hide();

    //1. init the tab control.
    initUis();

    //2. 读取配置文件
    ReadSettings();
    setWindowIcon(QIcon(":/images/robot.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    //3. 自动连接master
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    //4. 链接connect
    connections();

    //5.set window paramter
    setWindowFlags(Qt::FramelessWindowHint);
    ui.progressBar_acc->setStyleSheet("QProgressBar {border: 2px solid grey;\
                                                     border-radius: 5px;\
                                                     text-align: center;\
                                                     background:white;\
                                                     color: transparent;}"
                                      "QProgressBar::chunk{background-color:black;}");
    ui.progressBar_brake->setStyleSheet("QProgressBar {border: 2px solid grey;\
                                                 border-radius: 5px;\
                                                 text-align: center;\
                                                 background:white;\
                                                 color: transparent;}"
                                       "QProgressBar::chunk{background-color:black;}");
}

//订阅video话题
void MainWindow::initVideos()
{
    QSettings video_topic_setting("video_topic","novauto_monitor");
    QStringList names=video_topic_setting.value("names").toStringList();
    QStringList topics=video_topic_setting.value("topics").toStringList();

    if(topics.size()==4)
    {
        if(topics[0]!="")
            qnode.Sub_Image(topics[0],0);
    }

    //链接槽函数
    connect(&qnode,SIGNAL(Show_image(int,QImage)),this,SLOT(slot_show_image(int,QImage)));
}

// show camera topic
void MainWindow::slot_show_image(int frame_id, QImage image)
{
    switch (frame_id)
    {
    case 0:
        ui.label_video0->setPixmap(QPixmap::fromImage(image).scaled(ui.label_video0->width(),ui.label_video0->height()));
        break;
    case 1:
        //        ui.label_video1->setPixmap(QPixmap::fromImage(image).scaled(ui.label_video1->width(),ui.label_video1->height()));
        break;
    case 2:
        //        ui.label_video2->setPixmap(QPixmap::fromImage(image).scaled(ui.label_video2->width(),ui.label_video2->height()));
        break;
    case 3:
        //        ui.label_video3->setPixmap(QPixmap::fromImage(image).scaled(ui.label_video3->width(),ui.label_video3->height()));
        break;
    }
}

// fixed frame changed
void MainWindow::slot_fixed_frame_value_change(QString value) {
    map_rviz->SetGlobalOptions(value, QColor(250,48,48), 30);
}

// point cloud topic changed
void MainWindow::slot_pc_topic_value_change(QString value) {
    map_rviz->Display_PointCloud2(true, value);
}

//point cloud obstacle marker topic changed
void MainWindow::slot_pc_markerarray_topic_value_change(QString value) {
    map_rviz->Display_PointCloud2MarkerArray(true, value);
}

//radar obstacle marker topic changed
void MainWindow::slot_lrr_markerarray_topic_value_change(QString value) {
    map_rviz->Display_radar2MarkerArray(true, value);
}

//decision trajectory path topic changed
void MainWindow::slot_decision_trajectory_path_topic_value_change(QString value) {
    map_rviz->Display_DecisionTrajectoryPath(true, value);
}

//初始化UI
void MainWindow::initUis()
{
    ui.tab_manager->setTabEnabled(1,false);
    ui.tabWidget->setTabEnabled(1,false);
    ui.treeWidget_rviz->setEnabled(false);
    ui.tab_manager->setCurrentIndex(0);
    ui.tabWidget->setCurrentIndex(0);

    //velocity contorl
    m_DashBoard_x =new CCtrlDashBoard(ui.widget_speed_x);
    m_DashBoard_x->setGeometry(ui.widget_speed_x->rect());
    m_DashBoard_x->setValue(0);

    //yaw contorl
    steering_wheel =new YawCompass(ui.yaw);
    steering_wheel->setGeometry(ui.yaw->rect());

    //wheel
    wheel_ = new SteeringWheel(ui.wheel);
    wheel_->setGeometry(ui.wheel->rect());

    //treeWidget_rviz contorl
    ui.treeWidget_rviz->setWindowTitle("Displays");
    ui.treeWidget_rviz->setWindowIcon(QIcon("://images/classes/Displays.svg"));
    ui.treeWidget->setWindowTitle("Displays");
    ui.treeWidget->setWindowIcon(QIcon("://images/classes/Displays.svg"));

    //header 设置
    ui.treeWidget_rviz->setHeaderHidden(true);
    ui.treeWidget_rviz->setHeaderLabels(QStringList()<<"key"<<"value");
    ui.treeWidget->setHeaderHidden(true);
    ui.treeWidget->setHeaderLabels(QStringList()<<"key"<<"value");

    //1. fisrt tab 基本控制, add fixed frame.
    QTreeWidgetItem *Global_1 = new QTreeWidgetItem(QStringList()<<"Fixed Frame");
    Global_1 -> setIcon(0,QIcon("://images/options.png"));
    ui.treeWidget->addTopLevelItem(Global_1);
    QComboBox *frame_1=new QComboBox();
    frame_1->addItem("radarlrr");
    frame_1->setEditable(true);
    frame_1->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(Global_1, 1, frame_1);
    connect(frame_1,SIGNAL(currentTextChanged(QString)),this, SLOT(slot_fixed_frame_value_change(QString)));

    //1.1. pointcloud
//    QTreeWidgetItem *pointcloud_topic = new QTreeWidgetItem(QStringList()<<"PointCloud2");
//    pointcloud_topic -> setIcon(0,QIcon(":/images/classes/PointCloud2.png"));
//    ui.treeWidget->addTopLevelItem(pointcloud_topic);
//    QComboBox *topic_name=new QComboBox();
//    topic_name->addItem("/middle/rslidar_points");
//    topic_name->setEditable(true);
//    topic_name->setMaximumWidth(150);
//    topic_name->setLayoutDirection(Qt::LeftToRight);
//    ui.treeWidget->setItemWidget(pointcloud_topic, 1, topic_name);
//    connect(topic_name,SIGNAL(currentTextChanged(QString)),this, SLOT(slot_pc_topic_value_change(QString)));

    //1.1. radar obstacles marker
    QTreeWidgetItem *pointcloud_marker_topic1 = new QTreeWidgetItem(QStringList()<<"Label");
    pointcloud_marker_topic1 -> setIcon(0,QIcon(":/images/classes/MarkerArray.png"));
    ui.treeWidget->addTopLevelItem(pointcloud_marker_topic1);
    QComboBox *pointcloud_marker_topic_name1=new QComboBox();
    pointcloud_marker_topic_name1->addItem("/label");
    pointcloud_marker_topic_name1->setEditable(true);
    pointcloud_marker_topic_name1->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(pointcloud_marker_topic1, 1, pointcloud_marker_topic_name1);
    connect(pointcloud_marker_topic_name1,SIGNAL(currentTextChanged(QString)),this, SLOT(slot_lrr_markerarray_topic_value_change(QString)));

    //1.2. pointcloud obstacles marker
    QTreeWidgetItem *pointcloud_marker_topic2 = new QTreeWidgetItem(QStringList()<<"Perception");
    pointcloud_marker_topic2 -> setIcon(0,QIcon(":/images/classes/MarkerArray.png"));
    ui.treeWidget->addTopLevelItem(pointcloud_marker_topic2);
    QComboBox *pointcloud_marker_topic_name2=new QComboBox();
    pointcloud_marker_topic_name2->addItem("/markerArray");
    pointcloud_marker_topic_name2->setEditable(true);
    pointcloud_marker_topic_name2->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(pointcloud_marker_topic2, 1, pointcloud_marker_topic_name2);
    connect(pointcloud_marker_topic_name2,SIGNAL(currentTextChanged(QString)),this, SLOT(slot_pc_markerarray_topic_value_change(QString)));

    //1.3. decision trajectory path
    QTreeWidgetItem *decision_trajectory_path_topic = new QTreeWidgetItem(QStringList()<<"Planning");
    decision_trajectory_path_topic -> setIcon(0,QIcon(":/images/classes/Path.png"));
    ui.treeWidget->addTopLevelItem(decision_trajectory_path_topic);
    QComboBox *decision_trajectory_path_topic_name=new QComboBox();
    decision_trajectory_path_topic_name->addItem("/zzz/planning/decision_trajectory_path");
    decision_trajectory_path_topic_name->setEditable(true);
    decision_trajectory_path_topic_name->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(decision_trajectory_path_topic, 1, decision_trajectory_path_topic_name);
    connect(decision_trajectory_path_topic_name,SIGNAL(currentTextChanged(QString)),this, SLOT(slot_decision_trajectory_path_topic_value_change(QString)));

    //1.4. localization gps
    QTreeWidgetItem *localization_topic = new QTreeWidgetItem(QStringList()<<"GPS");
    localization_topic -> setIcon(0,QIcon(":/images/location.png"));
    ui.treeWidget->addTopLevelItem(localization_topic);
    QComboBox *localization_topic_name=new QComboBox();
    localization_topic_name->addItem("/localization/gps/fix");
    localization_topic_name->setEditable(true);
    localization_topic_name->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(localization_topic, 1, localization_topic_name);
    connect(localization_topic_name,SIGNAL(currentTextChanged(QString)),this, \
            SLOT(slot_localization_topic_value_change(QString)));

    //1.5. localization velocity
    QTreeWidgetItem *velocity_topic = new QTreeWidgetItem(QStringList()<<"Velocity");
    velocity_topic -> setIcon(0,QIcon(":/images/velocity.png"));
    ui.treeWidget->addTopLevelItem(velocity_topic);
    QComboBox *velocity_topic_name=new QComboBox();
    velocity_topic_name->addItem("/localization/gps/vel");
    velocity_topic_name->setEditable(true);
    velocity_topic_name->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(velocity_topic, 1, velocity_topic_name);
    connect(velocity_topic_name,SIGNAL(currentTextChanged(QString)),this, \
            SLOT(slot_velocity_topic_value_change(QString)));

    //1.6. localization imu
    QTreeWidgetItem *yaw_topic = new QTreeWidgetItem(QStringList()<<"IMU");
    yaw_topic -> setIcon(0,QIcon(":/images/classes/Odometry.png"));
    ui.treeWidget->addTopLevelItem(yaw_topic);
    QComboBox *yaw_topic_name=new QComboBox();
    yaw_topic_name->addItem("/localization/imu/data");
    yaw_topic_name->setEditable(true);
    yaw_topic_name->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(yaw_topic, 1, yaw_topic_name);
    connect(yaw_topic_name,SIGNAL(currentTextChanged(QString)),this, \
            SLOT(slot_imu_yaw_topic_value_change(QString)));

    //1.7. battery
    QTreeWidgetItem *battery_topic = new QTreeWidgetItem(QStringList()<<"Battery");
    battery_topic -> setIcon(0,QIcon(":/images/power-v.png"));
    ui.treeWidget->addTopLevelItem(battery_topic);
    QComboBox *battery_topic_name=new QComboBox();
    battery_topic_name->addItem("/CAN/BMS_fbk");
    battery_topic_name->setEditable(true);
    battery_topic_name->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(battery_topic, 1, battery_topic_name);
    connect(battery_topic_name,SIGNAL(currentTextChanged(QString)),this, \
            SLOT(slot_battery_topic_value_change(QString)));

    //1.8. gear status
    QTreeWidgetItem *gear_topic = new QTreeWidgetItem(QStringList()<<"Gear");
    gear_topic -> setIcon(0,QIcon(":/images/gear.png"));
    ui.treeWidget->addTopLevelItem(gear_topic);
    QComboBox *gear_topic_name=new QComboBox();
    gear_topic_name->addItem("/xp/auto_state");
    gear_topic_name->setEditable(true);
    gear_topic_name->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(gear_topic, 1, gear_topic_name);
    connect(gear_topic_name,SIGNAL(currentTextChanged(QString)),this, \
            SLOT(slot_gear_topic_value_change(QString)));

    //1.9. camera_0
    QTreeWidgetItem *camera_0_topic = new QTreeWidgetItem(QStringList()<<"Camera");
    camera_0_topic -> setIcon(0,QIcon(":/images/classes/Camera.png"));
    ui.treeWidget->addTopLevelItem(camera_0_topic);
    QComboBox *camera_0_topic_name=new QComboBox();
    camera_0_topic_name->addItem("/left_usb_cam/image_raw/compressed");
    camera_0_topic_name->setEditable(true);
    camera_0_topic_name->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(camera_0_topic, 1, camera_0_topic_name);
    connect(camera_0_topic_name,SIGNAL(currentTextChanged(QString)),this, \
            SLOT(slot_camera_0_topic_value_change(QString)));

    //1.10. pedal
    QTreeWidgetItem *pedal_topic = new QTreeWidgetItem(QStringList()<<"Pedal");
    pedal_topic -> setIcon(0,QIcon(":/images/pedal.png"));
    ui.treeWidget->addTopLevelItem(pedal_topic);
    QComboBox *pedal_topic_name=new QComboBox();
    pedal_topic_name->addItem("/canbus/SCU_IPC_6");
    pedal_topic_name->setEditable(true);
    pedal_topic_name->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(pedal_topic, 1, pedal_topic_name);
    connect(pedal_topic_name,SIGNAL(currentTextChanged(QString)),this, \
            SLOT(slot_pedal_topic_value_change(QString)));

    //1.11. wheel
    QTreeWidgetItem *wheel_topic = new QTreeWidgetItem(QStringList()<<"Wheel");
    wheel_topic -> setIcon(0,QIcon(":/images/wheel_logo.png"));
    ui.treeWidget->addTopLevelItem(wheel_topic);
    QComboBox *wheel_topic_name=new QComboBox();
    wheel_topic_name->addItem("/canbus/SCU_IPC_1");
    wheel_topic_name->setEditable(true);
    wheel_topic_name->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(wheel_topic, 1, wheel_topic_name);
    connect(wheel_topic_name,SIGNAL(currentTextChanged(QString)),this, \
            SLOT(slot_wheel_topic_value_change(QString)));

    //2. sensor status, default wrong.
    m_red_SheetStyle_ = "min-width: 16px; min-height: 16px;max-width:16px; max-height: 16px;border-radius: 8px;  border:1px solid black;background:red";
    m_green_SheetStyle_ = "min-width: 16px; min-height: 16px;max-width:16px; max-height: 16px;border-radius: 8px;  border:1px solid black;background:green";
    ui.lab_lidar_status->setStyleSheet(m_red_SheetStyle_);
    ui.lab_canbus_status->setStyleSheet(m_red_SheetStyle_);
    ui.lab_gnss_status->setStyleSheet(m_red_SheetStyle_);
    ui.lab_camera_status->setStyleSheet(m_red_SheetStyle_);
    ui.lab_localization_status->setStyleSheet(m_red_SheetStyle_);
    ui.lab_perception_status->setStyleSheet(m_red_SheetStyle_);
    ui.lab_control_status->setStyleSheet(m_red_SheetStyle_);
    ui.lab_planning_status->setStyleSheet(m_red_SheetStyle_);

    //3. rviz tab, Global options
    QTreeWidgetItem *Global=new QTreeWidgetItem(QStringList()<<"Global Options");
    Global->setIcon(0,QIcon("://images/options.png"));
    QTreeWidgetItem* FixedFrame=new QTreeWidgetItem(QStringList()<<"Fixed Frame");
    Global->addChild(FixedFrame);
    ui.treeWidget_rviz->addTopLevelItem(Global);
    Global->setExpanded(true);
    QComboBox *frame=new QComboBox();
    frame->addItem("radarlrr");
    frame->setEditable(true);
    frame->setMaximumWidth(150);
    ui.treeWidget_rviz->setItemWidget(FixedFrame,1,frame);

    QTreeWidgetItem* bcolor=new QTreeWidgetItem(QStringList()<<"Background Color");
    Global->addChild(bcolor);
    QLineEdit *colorval=new QLineEdit("4;255;48");
    colorval->setMaximumWidth(150);
    ui.treeWidget_rviz->setItemWidget(bcolor,1,colorval);

    QSpinBox *framerateval=new QSpinBox();
    framerateval->setStyleSheet("border:none");
    framerateval->setMaximumWidth(150);
    framerateval->setRange(10,50);
    framerateval->setValue(30);
    QTreeWidgetItem* framerate=new QTreeWidgetItem(QStringList()<<"Frame Rate");
    Global->addChild(framerate);
    ui.treeWidget_rviz->setItemWidget(framerate,1,framerateval);

    //3.1. grid
    QTreeWidgetItem *Grid=new QTreeWidgetItem(QStringList()<<"Grid");
    Grid->setIcon(0,QIcon("://images/classes/Grid.png"));
    ui.treeWidget_rviz->addTopLevelItem(Grid);
    Grid->setExpanded(true);
    QCheckBox* gridcheck=new QCheckBox;
    gridcheck->setChecked(true);
    ui.treeWidget_rviz->setItemWidget(Grid,1,gridcheck);

    QTreeWidgetItem *Grid_Status=new QTreeWidgetItem(QStringList()<<"Statue:");
    Grid_Status->setIcon(0,QIcon("://images/ok.png"));
    Grid->addChild(Grid_Status);
    QLabel *Grid_Status_Value=new QLabel("ok");
    Grid_Status_Value->setMaximumWidth(150);
    ui.treeWidget_rviz->setItemWidget(Grid_Status,1,Grid_Status_Value);

    QTreeWidgetItem* Reference_Frame=new QTreeWidgetItem(QStringList()<<"Reference Frame");
    QComboBox* Reference_Frame_Value=new QComboBox();
    Grid->addChild(Reference_Frame);
    Reference_Frame_Value->setMaximumWidth(150);
    Reference_Frame_Value->setEditable(true);
    Reference_Frame_Value->addItem("<Fixed Frame>");
    ui.treeWidget_rviz->setItemWidget(Reference_Frame,1,Reference_Frame_Value);

    QTreeWidgetItem* Plan_Cell_Count=new QTreeWidgetItem(QStringList()<<"Plan Cell Count");
    Grid->addChild(Plan_Cell_Count);
    QSpinBox* Plan_Cell_Count_Value=new QSpinBox();

    Plan_Cell_Count_Value->setMaximumWidth(150);
    Plan_Cell_Count_Value->setRange(1,100);
    Plan_Cell_Count_Value->setValue(10);
    ui.treeWidget_rviz->setItemWidget(Plan_Cell_Count,1,Plan_Cell_Count_Value);

    QTreeWidgetItem* Grid_Color=new QTreeWidgetItem(QStringList()<<"Color");
    QLineEdit* Grid_Color_Value=new QLineEdit();
    Grid_Color_Value->setMaximumWidth(150);
    Grid->addChild(Grid_Color);

    Grid_Color_Value->setText("160;160;160");
    ui.treeWidget_rviz->setItemWidget(Grid_Color,1,Grid_Color_Value);

    //qucik treewidget
    ui.treeWidget_quick_cmd->setHeaderLabels(QStringList()<<"key"<<"values");
    ui.treeWidget_quick_cmd->setHeaderHidden(true);
}

void MainWindow::initRviz()
{
    map_rviz=new QRviz(ui.verticalLayout_build_map,"qrviz");
    QComboBox *Global_op=(QComboBox *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(0),1);
    QString Reference_text=Global_op->currentText();
    map_rviz->Display_Grid(true,Reference_text,10,QColor(160,160,160));
    map_rviz->Display_PointCloud2(true, "/middle/rslidar_points");
    map_rviz->Display_PointCloud2MarkerArray(true, "/zzz/cognition/obstacles_markerarray");
    map_rviz->Display_DecisionTrajectoryPath(true, "/zzz/planning/decision_trajectory_path");
    map_rviz->SetGlobalOptions(Reference_text,QColor(250,250,250),30);
}

void MainWindow::connections()
{
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(slot_rosShutdown()));
    QObject::connect(&qnode, SIGNAL(Master_shutdown()), this, SLOT(slot_rosShutdown()));
    //绑定快捷按钮相关函数
    connect(ui.quick_cmd_add_btn,SIGNAL(clicked()),this,SLOT(quick_cmd_add()));
    connect(ui.quick_cmd_remove_btn,SIGNAL(clicked()),this,SLOT(quick_cmd_remove()));
    //设置界面
    connect(ui.action_2,SIGNAL(triggered(bool)),this,SLOT(slot_setting_frame()));
    //左工具栏tab索引改变
    connect(ui.tab_manager,SIGNAL(currentChanged(int)),this,SLOT(slot_tab_manage_currentChanged(int)));
    //右工具栏索引改变
    connect(ui.tabWidget,SIGNAL(currentChanged(int)),this,SLOT(slot_tab_Widget_currentChanged(int)));
    //刷新话题列表
    connect(ui.refreash_topic_btn,SIGNAL(clicked()),this,SLOT(refreashTopicList()));
    //添加rviz话题的按钮
    connect(ui.pushButton_add_topic,SIGNAL(clicked()),this,SLOT(slot_add_topic_btn()));
    //treewidget的值改变的槽函数
    //绑定treeiew所有控件的值改变函数
    for(int i=0;i<ui.treeWidget_rviz->topLevelItemCount();i++)
    {
        //top 元素
        QTreeWidgetItem *top=ui.treeWidget_rviz->topLevelItem(i);
        //        qDebug()<<top->text(0)<<endl;
        for(int j=0;j<top->childCount();j++)
        {
            //获取该WidgetItem的子节点
            QTreeWidgetItem* tmp= top->child(j);
            QWidget* controls=ui.treeWidget_rviz->itemWidget(tmp,1);
            //将当前控件对象和父级对象加入到map中
            widget_to_parentItem_map[controls]=top;
            //判断这些widget的类型 并分类型进行绑定槽函数
            if(QString(controls->metaObject()->className())=="QComboBox")
            {
                connect(controls,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
            }
            else if(QString(controls->metaObject()->className())=="QLineEdit")
            {
                connect(controls,SIGNAL(textChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
            }
            else if(QString(controls->metaObject()->className())=="QSpinBox")
            {
                connect(controls,SIGNAL(valueChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
            }
        }
    }

    //绑定treeiew所有控件的值改变函数
    for(int i=0;i<ui.treeWidget->topLevelItemCount();i++)
    {
        //top 元素
        QTreeWidgetItem *top=ui.treeWidget->topLevelItem(i);
        for(int j=0;j<top->childCount();j++)
        {
            //获取该WidgetItem的子节点
            QTreeWidgetItem* tmp= top->child(j);
            QWidget* controls=ui.treeWidget->itemWidget(tmp,1);
            //将当前控件对象和父级对象加入到map中
            widget_to_parentItem_map_first_tab[controls]=top;
            //判断这些widget的类型 并分类型进行绑定槽函数
            if(QString(controls->metaObject()->className())=="QComboBox")
            {
                connect(controls,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
            }
            else if(QString(controls->metaObject()->className())=="QLineEdit")
            {
                connect(controls,SIGNAL(textChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
            }
            else if(QString(controls->metaObject()->className())=="QSpinBox")
            {
                connect(controls,SIGNAL(valueChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
            }
        }
    }
    //绑定treeview checkbox选中事件
    // stateChanged
    for(int i=0;i<ui.treeWidget_rviz->topLevelItemCount();i++)
    {
        //top 元素
        QTreeWidgetItem *top=ui.treeWidget_rviz->topLevelItem(i);
        QWidget *check=ui.treeWidget_rviz->itemWidget(top,1);
        //记录父子关系
        widget_to_parentItem_map[check]=top;
        connect(check,SIGNAL(stateChanged(int)),this,SLOT(slot_treewidget_item_check_change(int)));
    }
    //connect(ui.treeWidget_rviz,SIGNAL(itemChanged(QTreeWidgetItem*,int)),this,SLOT(slot_treewidget_item_value_change(QTreeWidgetItem*,int)));
}

void MainWindow::on_settingButton_toggled(bool state)
{
    if(state) {
//        ui.tabWidget->setHidden(false);
        ui.tabWidget->setCurrentIndex(0);
    } else {
        ui.tabWidget->setCurrentIndex(4);
    }
}

void MainWindow::on_button_connect_clicked(bool check ) {
    //如果使用环境变量
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() ) {
            //showNoMasterMessage();
            QMessageBox::warning(NULL, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes , QMessageBox::Yes);
            ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
            ui.label_statue_text->setStyleSheet("color:red;");
            ui.label_statue_text->setText("离线");
            ui.treeWidget_rviz->setEnabled(false);
            ui.tab_manager->setTabEnabled(1,false);
            ui.tabWidget->setTabEnabled(1,false);
            ui.groupBox_3->setEnabled(false);
        } else {
            ui.tab_manager->setTabEnabled(1,true);
            ui.tabWidget->setTabEnabled(1,true);
            ui.groupBox_3->setEnabled(true);
            //初始化rviz
            initRviz();
            ui.treeWidget_rviz->setEnabled(true);
            ui.button_connect->setEnabled(false);
            ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
            ui.label_statue_text->setStyleSheet("color:green;");
            ui.label_statue_text->setText("在线");
            //显示话题列表
            initTopicList();

            ui.red_signal->setEnabled(true);
            ui.green_signal->setEnabled(true);
            //display map info.
            map_widget = new MapWidget(ui.graphicsView);
            map_widget->setGeometry(ui.graphicsView->rect());
            map_widget->setMouseTracking(true);
            map_widget->show();
            //电源的信号
            connect(&qnode,SIGNAL(Show_image(int,QImage)),this,SLOT(slot_show_image(int,QImage)));
            connect(&qnode,SIGNAL(power(int)),this,SLOT(slot_power(int)));
            connect(&qnode,SIGNAL(acc_pedal(int)),this,SLOT(slot_acc(int)));
            connect(&qnode,SIGNAL(brake_pedal(int)),this,SLOT(slot_brake(int)));
            connect(&qnode,SIGNAL(wheel_degree(float)),this,SLOT(slot_wheel_degree(float)));
            connect(&qnode,SIGNAL(Show_gps(double, double)),this,SLOT(slot_show_gps(double, double)));
            connect(&qnode,SIGNAL(Show_velocity(double, double)),this,SLOT(slot_show_velocity(double, double)));
            connect(&qnode,SIGNAL(Show_imu_data(double,double,double,double)),
                    this,SLOT(slot_show_imu_data(double,double,double,double)));
            connect(&qnode,SIGNAL(sensor_self_check(int,int,int,int,int,int,int,int)),this,\
                    SLOT(slot_show_sensor_status(int,int,int,int,int,int,int,int)));

        }
    }
    //如果不使用环境变量
    else {
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                          ui.line_edit_host->text().toStdString()) )
        {
            QMessageBox::warning(NULL, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes , QMessageBox::Yes);
            ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
            ui.label_statue_text->setStyleSheet("color:red;");
            ui.label_statue_text->setText("离线");
            ui.treeWidget_rviz->setEnabled(false);
            ui.tab_manager->setTabEnabled(1,false);
            ui.tabWidget->setTabEnabled(1,false);
            ui.groupBox_3->setEnabled(false);
            //            showNoMasterMessage();
        }
        else
        {
            ui.tab_manager->setTabEnabled(1,true);
            ui.tabWidget->setTabEnabled(1,true);
            //初始化rviz
            initRviz();
            ui.treeWidget_rviz->setEnabled(true);
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.groupBox_3->setEnabled(true);
            ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
            ui.label_statue_text->setStyleSheet("color:green;");
            ui.label_statue_text->setText("在线");
            //显示话题列表
            initTopicList();

            ui.red_signal->setEnabled(true);
            ui.green_signal->setEnabled(true);
            //display map info.
            map_widget = new MapWidget(ui.graphicsView);
            map_widget->setGeometry(ui.graphicsView->rect());
            map_widget->setMouseTracking(true);
            map_widget->show();
            //电源的信号
            connect(&qnode,SIGNAL(Show_image(int,QImage)),this,SLOT(slot_show_image(int,QImage)));
            connect(&qnode,SIGNAL(power(int)),this,SLOT(slot_power(int)));
            connect(&qnode,SIGNAL(acc_pedal(int)),this,SLOT(slot_acc(int)));
            connect(&qnode,SIGNAL(brake_pedal(int)),this,SLOT(slot_brake(int)));
            connect(&qnode,SIGNAL(wheel_degree(float)),this,SLOT(slot_wheel_degree(float)));
            connect(&qnode,SIGNAL(Show_gps(double, double)),this,SLOT(slot_show_gps(double, double)));
            connect(&qnode,SIGNAL(Show_velocity(double, double)),this,SLOT(slot_show_velocity(double, double)));
            connect(&qnode,SIGNAL(Show_imu_data(double,double,double,double)),
                    this,SLOT(slot_show_imu_data(double,double,double,double)));
            connect(&qnode,SIGNAL(sensor_self_check(int,int,int,int,int,int,int,int)),this,\
                    SLOT(slot_show_sensor_status(int,int,int,int,int,int,int,int)));
            connect(&qnode,SIGNAL(Show_gear_status(int)),this,SLOT(slot_show_gear_status(int)));
        }
    }
}

void MainWindow::slot_show_sensor_status(int lidar, int canbus, int gnss, int camera,\
                                 int localization, int perception, int control, int planning)
{
    if(lidar)
        ui.lab_lidar_status->setStyleSheet(m_green_SheetStyle_);
    else
        ui.lab_lidar_status->setStyleSheet(m_red_SheetStyle_);
    if(canbus)
        ui.lab_canbus_status->setStyleSheet(m_green_SheetStyle_);
    else
        ui.lab_canbus_status->setStyleSheet(m_red_SheetStyle_);
    if(gnss)
        ui.lab_gnss_status->setStyleSheet(m_green_SheetStyle_);
    else
        ui.lab_gnss_status->setStyleSheet(m_red_SheetStyle_);
    if(camera)
        ui.lab_camera_status->setStyleSheet(m_green_SheetStyle_);
    else
        ui.lab_camera_status->setStyleSheet(m_red_SheetStyle_);
    if(localization)
        ui.lab_localization_status->setStyleSheet(m_green_SheetStyle_);
    else
    {
        ui.lab_localization_status->setStyleSheet(m_red_SheetStyle_);
        m_DashBoard_x->setValue(0);
        ui.lab_yaw->setText(QString::number(0, 'f', 0));
        steering_wheel->setValue(0);
        map_widget -> car_degree_ = 0;
    }
    if(perception)
        ui.lab_perception_status->setStyleSheet(m_green_SheetStyle_);
    else
        ui.lab_perception_status->setStyleSheet(m_red_SheetStyle_);
    if(control)
        ui.lab_control_status->setStyleSheet(m_green_SheetStyle_);
    else
        ui.lab_control_status->setStyleSheet(m_red_SheetStyle_);
    if(planning)
        ui.lab_planning_status->setStyleSheet(m_green_SheetStyle_);
    else
        ui.lab_planning_status->setStyleSheet(m_red_SheetStyle_);
}

void MainWindow::on_button_disconnect_clicked(bool check )
{
    emit qnode.rosShutdown();
}

void MainWindow::on_red_signal_clicked(bool check)
{
    ui.green_signal->setIcon(QIcon(""));
    ui.red_signal->setIcon(QIcon(":/images/red.png"));
    //    zzz_perception_msgs::DetectionBoxArray traffic_light_detection;
    //    zzz_perception_msgs::DetectionBox traffic_light;
    //    traffic_light.signal.flags = zzz_perception_msgs::ObjectSignals::TRAFFIC_LIGHT_RED;
    //    traffic_light_detection.detections.push_back(traffic_light);
    //    qnode.pub_traffic_light.publish(traffic_light_detection);
    //    qDebug()<<"### Send RED Signal....";
    QTimeLine *timeline=new QTimeLine(1000);
    timeline->setFrameRange(0, 11);
    connect(timeline,SIGNAL(frameChanged(int)),this,SLOT(red_sender()));
    timeline->start();
}

void MainWindow::red_sender()
{
    zzz_perception_msgs::DetectionBoxArray traffic_light_detection;
    zzz_perception_msgs::DetectionBox traffic_light;
    traffic_light.signal.flags = zzz_perception_msgs::ObjectSignals::TRAFFIC_LIGHT_RED;
    traffic_light_detection.detections.push_back(traffic_light);
    qnode.pub_traffic_light.publish(traffic_light_detection);
    qDebug()<<"### Send RED Signal....";
}

void MainWindow::on_green_signal_clicked(bool check)
{
    ui.green_signal->setIcon(QIcon(":/images/green.png"));
    ui.red_signal->setIcon(QIcon(""));
    //    zzz_perception_msgs::DetectionBoxArray traffic_light_detection;
    //    zzz_perception_msgs::DetectionBox traffic_light;
    //    traffic_light.signal.flags = zzz_perception_msgs::ObjectSignals::TRAFFIC_LIGHT_GREEN;
    //    traffic_light_detection.detections.push_back(traffic_light);
    //    qnode.pub_traffic_light.publish(traffic_light_detection);
    //    qDebug()<<"### Send GREEN Signal....";
    QTimeLine *timeline=new QTimeLine(1000);
    timeline->setFrameRange(0, 11);
    connect(timeline,SIGNAL(frameChanged(int)),this,SLOT(green_sender()));
    timeline->start();
}

void MainWindow::green_sender()
{
    zzz_perception_msgs::DetectionBoxArray traffic_light_detection;
    zzz_perception_msgs::DetectionBox traffic_light;
    traffic_light.signal.flags = zzz_perception_msgs::ObjectSignals::TRAFFIC_LIGHT_GREEN;
    traffic_light_detection.detections.push_back(traffic_light);
    qnode.pub_traffic_light.publish(traffic_light_detection);
    qDebug()<<"### Send GREEN Signal....";
}

//camera
void MainWindow::slot_camera_0_topic_value_change(QString topic_name)
{
    qnode.Sub_Image(topic_name);
}

//localization gps
void MainWindow::slot_localization_topic_value_change(QString topic_name)
{
    qnode.Sub_Gps(topic_name);
}
void MainWindow::slot_show_gps(double lon, double lat) {
    map_widget -> paintGps(lon, lat);
}

//localization velocity
void MainWindow::slot_velocity_topic_value_change(QString topic_name)
{
    qnode.Sub_Velocity(topic_name);
}
void MainWindow::slot_show_velocity(double vel_x, double vel_y)
{
    double current_vel = std::sqrt(std::pow(vel_x, 2) + std::pow(vel_y, 2));
    //    m_DashBoard_x->setValue(qAbs(current_vel)-1); // m/s
    m_DashBoard_x->setValue(qAbs(current_vel)*3.6-1); // km/h
}

//localization imu yaw
void MainWindow::slot_imu_yaw_topic_value_change(QString topic_name)
{
    qnode.Sub_Imu(topic_name);
}
void MainWindow::slot_show_imu_data(double x, double y, double z, double w)
{
    double yaw_rad = qAtan2(2*(w*z+x*y),1-2*(y*y+z*z));
    //    double roll, pitch, yaw;
    //    tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
    double yaw_deg = -qRadiansToDegrees(yaw_rad);
    //    qDebug()<<yaw_rad<<"   "<<yaw;
    ui.lab_yaw->setText(QString::number(yaw_deg + 90, 'f', 4));
    steering_wheel->setValue(yaw_deg + 90); // km/h
    map_widget -> car_degree_ = yaw_deg + 90;
}

//Battery
void MainWindow::slot_battery_topic_value_change(QString topic_name)
{
    qnode.Sub_Power(topic_name);
}
void MainWindow::slot_power(int p)
{
    ui.progressBar->setValue(p>100?100:p);
    //当电量过低时发出提示
    if(p <= 10)
    {
         ui.progressBar->setStyleSheet("QProgressBar::chunk {background-color: red;width: 20px;} QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");
          // QMessageBox::warning(NULL, "电量不足", "电量不足，请及时充电！", QMessageBox::Yes , QMessageBox::Yes);
    }
    else
    {
        ui.progressBar->setStyleSheet("QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: center;}");
    }
}

void MainWindow::slot_acc(int p)
{
    ui.progressBar_acc->setValue(p>100?100:p);
}

void MainWindow::slot_brake(int p)
{
    ui.progressBar_brake->setValue(p>100?100:p);
}

void MainWindow::slot_pedal_topic_value_change(QString topic_name)
{
    qnode.Sub_Acc_Brk_Ped(topic_name);
}

void MainWindow::slot_wheel_degree(float p)
{
    wheel_ -> setValue(p);
}

void MainWindow::slot_wheel_topic_value_change(QString topic_name)
{
    qnode.Sub_wheel_Deg(topic_name);
}

//CAN gear status
void MainWindow::slot_gear_topic_value_change(QString topic_name)
{
    qnode.Sub_Gear_Status(topic_name);
}

void MainWindow::slot_show_gear_status(int gear_status)
{
    if(gear_status == 1) {
        ui.label_gear_status->setText("D");
    } else if (gear_status == 2) {
        ui.label_gear_status->setText("N");
    } else if (gear_status == 3) {
        ui.label_gear_status->setText("R");
    } else if (gear_status == 4) {
        ui.label_gear_status->setText("P");
    }
}

void MainWindow::initTopicList()
{
    ui.topic_listWidget->clear();
    ui.topic_listWidget->addItem(QString("%1   (%2)").arg("Name","Type"));
    QMap<QString,QString> topic_list= qnode.get_topic_list();
    for(QMap<QString,QString>::iterator iter=topic_list.begin();iter!=topic_list.end();iter++)
    {
        ui.topic_listWidget->addItem(QString("%1   (%2)").arg(iter.key(),iter.value()));
    }
}

void MainWindow::refreashTopicList()
{
    initTopicList();
}

//当ros与master的连接断开时
void MainWindow::slot_rosShutdown()
{
    ui.label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
    ui.label_statue_text->setStyleSheet("color:red;");
    ui.label_statue_text->setText("离线");
    ui.button_connect->setEnabled(true);
    ui.line_edit_master->setReadOnly(false);
    ui.line_edit_host->setReadOnly(false);
}

//设置界面
void MainWindow::slot_setting_frame()
{
    if(set!=NULL)
    {
        delete set;
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    else{
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    //绑定set确认按钮点击事件
}

//treewidget的checkbox是否选中槽函数
void MainWindow::slot_treewidget_item_check_change(int is_check)
{
    QCheckBox* sen = (QCheckBox*)sender();
    qDebug()<<"check:"<<is_check<<"parent:"<<widget_to_parentItem_map[sen]->text(0)<<"地址："<<widget_to_parentItem_map[sen];
    QTreeWidgetItem *parentItem=widget_to_parentItem_map[sen];
    QString dis_name=widget_to_parentItem_map[sen]->text(0);
    bool enable=is_check>1?true:false;
    if(dis_name=="Grid")
    {
        QLineEdit *Color_text=(QLineEdit *) ui.treeWidget_rviz->itemWidget(parentItem->child(3),1);
        QString co=Color_text->text();
        QStringList colorList=co.split(";");
        QColor cell_color=QColor(colorList[0].toInt(),colorList[1].toInt(),colorList[2].toInt());

        QComboBox *Reference_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        QString Reference_text=Reference_box->currentText();
        if(Reference_box->currentText()=="<Fixed Frame>")
        {
            QComboBox *Global_op=(QComboBox *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(0),1);
            Reference_text=Global_op->currentText();
        }
        QSpinBox *plan_cell_count=(QSpinBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(2),1);
        map_rviz->Display_Grid(enable,Reference_text,plan_cell_count->text().toInt(),cell_color);

    }
    else if(dis_name=="Map")
    {
        QComboBox *topic_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        QLineEdit *alpha=(QLineEdit *) ui.treeWidget_rviz->itemWidget(parentItem->child(2),1);
        QComboBox *scheme=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(3),1);
        map_rviz->Display_Map(enable,topic_box->currentText(),alpha->text().toDouble(),scheme->currentText());
    }
    else if(dis_name=="LaserScan")
    {
        QComboBox *topic_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        map_rviz->Display_LaserScan(enable,topic_box->currentText());
    }
    else if(dis_name=="PointCloud2")
    {
        QComboBox *topic_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        map_rviz->Display_PointCloud2(enable,topic_box->currentText());
    }
    else if(dis_name=="Navigate")
    {
        QComboBox* Global_map=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(0)->child(0)->child(0),1);
        QComboBox* Global_plan=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(0)->child(1)->child(0),1);
        QComboBox* Local_map=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1)->child(0)->child(0),1);
        QComboBox* Local_plan=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1)->child(1)->child(0),1);
        map_rviz->Display_Navigate(enable,Global_map->currentText(),Global_plan->currentText(),Local_map->currentText(),Local_plan->currentText());
    }
    else if(dis_name=="RobotModel")
    {
        map_rviz->Display_RobotModel(enable);
    }
}

//treewidget 的值改变槽函数
void MainWindow::slot_treewidget_item_value_change(QString value)
{
    QWidget* sen = (QWidget*)sender();
    QTreeWidgetItem *parentItem=widget_to_parentItem_map[sen];
    QString Dis_Name=widget_to_parentItem_map[sen]->text(0);
    //判断每种显示的类型
    if(Dis_Name=="Grid")
    {
        //是否启用该图层
        QCheckBox *che_box=(QCheckBox *) ui.treeWidget_rviz->itemWidget(parentItem,1);
        bool enable=che_box->isChecked();
        QLineEdit *Color_text=(QLineEdit *) ui.treeWidget_rviz->itemWidget(parentItem->child(3),1);
        QString co=Color_text->text();
        QStringList colorList=co.split(";");
        QColor cell_color=QColor(colorList[0].toInt(),colorList[1].toInt(),colorList[2].toInt());

        QComboBox *Reference_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        QString Reference_text=Reference_box->currentText();
        if(Reference_box->currentText()=="<Fixed Frame>")
        {
            QComboBox *Global_op=(QComboBox *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(0),1);
            Reference_text=Global_op->currentText();
        }
        QSpinBox *plan_cell_count=(QSpinBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(2),1);
        map_rviz->Display_Grid(enable,Reference_text,plan_cell_count->text().toInt(),cell_color);

    }
    else if(Dis_Name=="Global Options")
    {
        QComboBox *Global_op=(QComboBox *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(0),1);
        QString Reference_text=Global_op->currentText();
        QLineEdit *back_color=(QLineEdit *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(1),1);
        QStringList coList=back_color->text().split(";");
        QColor colorBack=QColor(coList[0].toInt(),coList[1].toInt(),coList[2].toInt());
        std::cout << "Helloworld" << std::endl;
        QSpinBox *FrameRaBox=(QSpinBox *) ui.treeWidget_rviz->itemWidget(ui.treeWidget_rviz->topLevelItem(0)->child(2),1);
        map_rviz->SetGlobalOptions(Reference_text,colorBack,FrameRaBox->value());
    }
    else if(Dis_Name=="Map")
    {
        //是否启用该图层
        QCheckBox *che_box=(QCheckBox *) ui.treeWidget_rviz->itemWidget(parentItem,1);
        bool enable=che_box->isChecked();
        QComboBox *topic_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        QLineEdit *alpha=(QLineEdit *) ui.treeWidget_rviz->itemWidget(parentItem->child(2),1);
        QComboBox *scheme=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(3),1);
        qDebug()<<topic_box->currentText()<<alpha->text()<<scheme->currentText();
        map_rviz->Display_Map(enable,topic_box->currentText(),alpha->text().toDouble(),scheme->currentText());
    }
    else if(Dis_Name=="LaserScan")
    {
        //是否启用该图层
        QCheckBox *che_box=(QCheckBox *) ui.treeWidget_rviz->itemWidget(parentItem,1);
        bool enable=che_box->isChecked();
        QComboBox *topic_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        map_rviz->Display_LaserScan(enable,topic_box->currentText());
    }
    else if(Dis_Name=="PointCloud2")
    {
        //是否启用该图层
        QCheckBox *che_box=(QCheckBox *) ui.treeWidget_rviz->itemWidget(parentItem,1);
        bool enable=che_box->isChecked();
        QComboBox *topic_box=(QComboBox *) ui.treeWidget_rviz->itemWidget(parentItem->child(1),1);
        map_rviz->Display_PointCloud2(enable,topic_box->currentText());
    }
}

//rviz添加topic的槽函数
void MainWindow::slot_add_topic_btn()
{
    if(!addtopic_form)
    {
        addtopic_form=new AddTopics();
        //阻塞其他窗体
        addtopic_form->setWindowModality(Qt::ApplicationModal);
        //绑定添加rviz话题信号
        connect(addtopic_form,SIGNAL(Topic_choose(QTreeWidgetItem *)),this,SLOT(slot_choose_topic(QTreeWidgetItem *)));
        addtopic_form->show();
    } else{
        QPoint p=addtopic_form->pos();
        delete addtopic_form;
        addtopic_form=new AddTopics();
        connect(addtopic_form,SIGNAL(Topic_choose(QTreeWidgetItem *)),this,SLOT(slot_choose_topic(QTreeWidgetItem *)));
        addtopic_form->show();
        addtopic_form->move(p.x(),p.y());
    }
}

//选中要添加的话题的槽函数
void MainWindow::slot_choose_topic(QTreeWidgetItem *choose)
{
    ui.treeWidget_rviz->addTopLevelItem(choose);
    //添加是否启用的checkbox
    QCheckBox *check=new QCheckBox();
    ui.treeWidget_rviz->setItemWidget(choose,1,check);
    //记录父子关系
    widget_to_parentItem_map[check]=choose;
    //绑定checkbox的槽函数
    connect(check,SIGNAL(stateChanged(int)),this,SLOT(slot_treewidget_item_check_change(int)));
    //添加状态的对应关系到map
    tree_rviz_stues[choose->text(0)]=choose->child(0);

    if(choose->text(0)=="Map")
    {
        QComboBox *Map_Topic=new QComboBox();
        Map_Topic->addItem("map");
        Map_Topic->setEditable(true);
        Map_Topic->setMaximumWidth(150);
        widget_to_parentItem_map[Map_Topic]=choose;
        ui.treeWidget_rviz->setItemWidget(choose->child(1),1,Map_Topic);
        //绑定值改变了的事件
        connect(Map_Topic,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        QLineEdit *map_alpha=new QLineEdit;
        map_alpha->setMaximumWidth(150);
        map_alpha->setText("0.7");
        widget_to_parentItem_map[map_alpha]=choose;
        //绑定值改变了的事件
        connect(map_alpha,SIGNAL(textChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        ui.treeWidget_rviz->setItemWidget(choose->child(2),1,map_alpha);

        QComboBox *Map_Scheme=new QComboBox;
        Map_Scheme->setMaximumWidth(150);
        Map_Scheme->addItems(QStringList()<<"map"<<"costmap"<<"raw");
        widget_to_parentItem_map[Map_Scheme]=choose;
        //绑定值改变了的事件
        connect(Map_Scheme,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        ui.treeWidget_rviz->setItemWidget(choose->child(3),1,Map_Scheme);
    }
    else if(choose->text(0)=="LaserScan")
    {
        QComboBox *Laser_Topic=new QComboBox;
        Laser_Topic->setMaximumWidth(150);
        Laser_Topic->addItem("scan");
        Laser_Topic->setEditable(true);
        widget_to_parentItem_map[Laser_Topic]=choose;
        ui.treeWidget_rviz->setItemWidget(choose->child(1),1,Laser_Topic);
        //绑定值改变了的事件
        connect(Laser_Topic,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
    }
    else if(choose->text(0)=="PointCloud2")
    {
        QComboBox *PointCloud2_Topic=new QComboBox();
        PointCloud2_Topic->setMaximumWidth(150);
        PointCloud2_Topic->addItem("/middle/rslidar_points");
        PointCloud2_Topic->setEditable(true);
        widget_to_parentItem_map[PointCloud2_Topic]=choose;
        ui.treeWidget_rviz->setItemWidget(choose->child(1),1,PointCloud2_Topic);
        //绑定值改变了的事件
        connect(PointCloud2_Topic,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));

    }
    else if(choose->text(0)=="Navigate")
    {
        //Global Map
        QTreeWidgetItem *Global_map=new QTreeWidgetItem(QStringList()<<"Global Map");
        Global_map->setIcon(0,QIcon("://images/classes/Group.png"));
        choose->addChild(Global_map);
        QTreeWidgetItem *Costmap=new QTreeWidgetItem(QStringList()<<"Costmap");
        Costmap->setIcon(0,QIcon("://images/classes/Map.png"));
        QTreeWidgetItem *Costmap_topic=new QTreeWidgetItem(QStringList()<<"Topic");
        Costmap->addChild(Costmap_topic);
        QComboBox *Costmap_Topic_Vel=new QComboBox();
        Costmap_Topic_Vel->setEditable(true);
        Costmap_Topic_Vel->setMaximumWidth(150);
        Costmap_Topic_Vel->addItem("/move_base/global_costmap/costmap");
        ui.treeWidget_rviz->setItemWidget(Costmap_topic,1,Costmap_Topic_Vel);
        Global_map->addChild(Costmap);
        //绑定子父关系
        widget_to_parentItem_map[Costmap_Topic_Vel]=choose;
        //绑定值改变了的事件
        connect(Costmap_Topic_Vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));

        QTreeWidgetItem* CostMap_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
        CostMap_Planner->setIcon(0,QIcon("://images/classes/Path.png"));
        Global_map->addChild(CostMap_Planner);

        QTreeWidgetItem* CostMap_Planner_topic=new QTreeWidgetItem(QStringList()<<"Topic");
        QComboBox* Costmap_Planner_Topic_Vel=new QComboBox();
        Costmap_Planner_Topic_Vel->setMaximumWidth(150);
        Costmap_Planner_Topic_Vel->addItem("/move_base/DWAPlannerROS/global_plan");
        Costmap_Planner_Topic_Vel->setEditable(true);
        CostMap_Planner->addChild(CostMap_Planner_topic);
        ui.treeWidget_rviz->setItemWidget(CostMap_Planner_topic,1,Costmap_Planner_Topic_Vel);
        //绑定子父关系
        widget_to_parentItem_map[Costmap_Planner_Topic_Vel]=choose;
        //绑定值改变了的事件
        connect(Costmap_Planner_Topic_Vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));

        //Local Map
        QTreeWidgetItem *Local_map=new QTreeWidgetItem(QStringList()<<"Local Map");
        Local_map->setIcon(0,QIcon("://images/classes/Group.png"));
        choose->addChild(Local_map);

        QTreeWidgetItem *Local_Costmap=new QTreeWidgetItem(QStringList()<<"Costmap");
        Local_Costmap->setIcon(0,QIcon("://images/classes/Map.png"));
        Local_map->addChild(Local_Costmap);

        QTreeWidgetItem *local_costmap_topic=new QTreeWidgetItem(QStringList()<<"Topic");
        Local_Costmap->addChild(local_costmap_topic);
        QComboBox *local_costmap_topic_vel=new QComboBox();
        local_costmap_topic_vel->setEditable(true);
        local_costmap_topic_vel->setMaximumWidth(150);
        local_costmap_topic_vel->addItem("/move_base/local_costmap/costmap");
        ui.treeWidget_rviz->setItemWidget(local_costmap_topic,1,local_costmap_topic_vel);

        //绑定子父关系
        widget_to_parentItem_map[local_costmap_topic_vel]=choose;
        //绑定值改变了的事件
        connect(local_costmap_topic_vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));

        QTreeWidgetItem* LocalMap_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
        LocalMap_Planner->setIcon(0,QIcon("://images/classes/Path.png"));
        Local_map->addChild(LocalMap_Planner);

        QTreeWidgetItem* Local_Planner_topic=new QTreeWidgetItem(QStringList()<<"Topic");

        QComboBox* Local_Planner_Topic_Vel=new QComboBox();
        Local_Planner_Topic_Vel->setMaximumWidth(150);
        Local_Planner_Topic_Vel->addItem("/move_base/DWAPlannerROS/local_plan");
        Local_Planner_Topic_Vel->setEditable(true);
        LocalMap_Planner->addChild(Local_Planner_topic);
        ui.treeWidget_rviz->setItemWidget(Local_Planner_topic,1, Local_Planner_Topic_Vel);
        //绑定子父关系
        widget_to_parentItem_map[Local_Planner_Topic_Vel]=choose;
        //绑定值改变了的事件
        connect(Local_Planner_Topic_Vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        //CostCloud
        QTreeWidgetItem* CostCloud=new QTreeWidgetItem(QStringList()<<"Cost Cloud");
        CostCloud->setIcon(0,QIcon("://images/classes/PointCloud2.png"));
        Local_map->addChild(CostCloud);
        QTreeWidgetItem *CostCloud_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
        QComboBox* CostCloud_Topic_Vel=new QComboBox();
        CostCloud_Topic_Vel->setMaximumWidth(150);
        CostCloud_Topic_Vel->setEditable(true);
        CostCloud_Topic_Vel->addItem("/move_base/DWAPlannerROS/cost_cloud");
        CostCloud->addChild(CostCloud_Topic);
        ui.treeWidget_rviz->setItemWidget(CostCloud_Topic,1,CostCloud_Topic_Vel);
        //绑定子父关系
        widget_to_parentItem_map[CostCloud_Topic_Vel]=CostCloud;
        //绑定值改变了的事件
        connect(CostCloud_Topic_Vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        //Trajectory Cloud
        QTreeWidgetItem* TrajectoryCloud=new QTreeWidgetItem(QStringList()<<"Trajectory Cloud");
        TrajectoryCloud->setIcon(0,QIcon("://images/classes/PointCloud2.png"));
        Local_map->addChild(TrajectoryCloud);
        QTreeWidgetItem *TrajectoryCloud_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
        QComboBox* TrajectoryCloud_Topic_Vel=new QComboBox();
        TrajectoryCloud_Topic_Vel->setMaximumWidth(150);
        TrajectoryCloud_Topic_Vel->setEditable(true);
        TrajectoryCloud_Topic_Vel->addItem("/move_base/DWAPlannerROS/trajectory_cloud");
        TrajectoryCloud->addChild(TrajectoryCloud_Topic);
        ui.treeWidget_rviz->setItemWidget(TrajectoryCloud_Topic,1,TrajectoryCloud_Topic_Vel);
        //绑定子父关系
        widget_to_parentItem_map[TrajectoryCloud_Topic_Vel]=TrajectoryCloud;
        //绑定值改变了的事件
        connect(TrajectoryCloud_Topic_Vel,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_item_value_change(QString)));
        ui.treeWidget_rviz->addTopLevelItem(choose);
        choose->setExpanded(true);
    }
    else if(choose->text(0)=="TF")
    {
        ui.treeWidget_rviz->addTopLevelItem(choose);
    }

    //默认选中
    check->setChecked(true);
}

//左工具栏索引改变
void MainWindow::slot_tab_manage_currentChanged(int index)
{
    switch (index) {
    case 0:
        ui.tabWidget->setCurrentIndex(0);
        break;
    case 1:
        ui.tabWidget->setCurrentIndex(1);
        break;
    case 2:
        ui.tabWidget->setCurrentIndex(2);
        break;
    }
}

//右工具栏索引改变
void MainWindow::slot_tab_Widget_currentChanged(int index)
{
    switch (index) {
    case 0:
        ui.tab_manager->setCurrentIndex(0);
        break;
    case 1:
        ui.tab_manager->setCurrentIndex(1);
        break;
    case 2:
        ui.tab_manager->setCurrentIndex(2);
        break;
    }
}

//快捷指令删除按钮
void MainWindow::quick_cmd_remove()
{
    QTreeWidgetItem *curr=ui.treeWidget_quick_cmd->currentItem();
    //没有选择节点
    if(curr==NULL) return;
    //获取父节点
    QTreeWidgetItem* parent=curr->parent();
    //如果当前节点就为父节点
    if(parent==NULL)
    {
        ui.treeWidget_quick_cmd->takeTopLevelItem(ui.treeWidget_quick_cmd->indexOfTopLevelItem(curr));
        delete curr;
    }
    else{
        ui.treeWidget_quick_cmd->takeTopLevelItem(ui.treeWidget_quick_cmd->indexOfTopLevelItem(parent));
        delete parent;
    }
}

//快捷指令添加按钮
void MainWindow::quick_cmd_add()
{
    QWidget *w=new QWidget;
    //阻塞其他窗体
    w->setWindowModality(Qt::ApplicationModal);
    QLabel *name=new QLabel;
    name->setText("名称:");
    QLabel *content=new QLabel;
    content->setText("脚本:");
    QLineEdit *name_val=new QLineEdit;
    QTextEdit *shell_val=new QTextEdit;
    QPushButton *ok_btn=new QPushButton;
    ok_btn->setText("ok");
    ok_btn->setIcon(QIcon("://images/ok.png"));
    QPushButton *cancel_btn=new QPushButton;
    cancel_btn->setText("cancel");
    cancel_btn->setIcon(QIcon("://images/false.png"));
    QHBoxLayout *lay1=new QHBoxLayout;
    lay1->addWidget(name);
    lay1->addWidget(name_val);
    QHBoxLayout *lay2=new QHBoxLayout;
    lay2->addWidget(content);
    lay2->addWidget(shell_val);
    QHBoxLayout *lay3=new QHBoxLayout;
    lay3->addWidget(ok_btn);
    lay3->addWidget(cancel_btn);
    QVBoxLayout *v1=new QVBoxLayout;
    v1->addLayout(lay1);
    v1->addLayout(lay2);
    v1->addLayout(lay3);

    w->setLayout(v1);
    w->show();

    connect(ok_btn,&QPushButton::clicked,[this,w,name_val,shell_val]
    {
        this->add_quick_cmd(name_val->text(),shell_val->toPlainText());
        w->close();
    });
}

//向treeWidget添加快捷指令
void MainWindow::add_quick_cmd(QString name,QString val)
{
    if(name=="")
        return;
    QTreeWidgetItem *head=new QTreeWidgetItem(QStringList()<<name);
    this->ui.treeWidget_quick_cmd->addTopLevelItem(head);
    QCheckBox *check=new QCheckBox;
    //记录父子关系
    this->widget_to_parentItem_map[check]=head;
    //连接checkbox选中的槽函数
    connect(check,SIGNAL(stateChanged(int)),this,SLOT(quick_cmds_check_change(int)));
    this->ui.treeWidget_quick_cmd->setItemWidget(head,1,check);
    QTreeWidgetItem *shell_content=new QTreeWidgetItem(QStringList()<<"shell");
    QTextEdit *shell_val=new QTextEdit;
    shell_val->setMaximumWidth(150);
    shell_val->setMaximumHeight(40);
    head->addChild(shell_content);
    shell_val->setText(val);
    this->ui.treeWidget_quick_cmd->setItemWidget(shell_content,1,shell_val);
}

//快捷指令按钮处理的函数
void MainWindow::quick_cmds_check_change(int state)
{
    QCheckBox* check = qobject_cast<QCheckBox*>(sender());
    QTreeWidgetItem *parent=widget_to_parentItem_map[check];
    QString bash=((QTextEdit *)ui.treeWidget_quick_cmd->itemWidget(parent->child(0),1))->toPlainText();
    bool is_checked=state>1?true:false;
    if(is_checked)
    {
        quick_cmd=new QProcess;
        quick_cmd->start("bash");
        qDebug()<<bash;
        quick_cmd->write(bash.toLocal8Bit()+'\n');
        connect(quick_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(cmd_output()));
        connect(quick_cmd,SIGNAL(readyReadStandardError()),this,SLOT(cmd_error_output()));
    }
    else{
        qDebug()<<"release";
        quick_cmd->kill();
    }
}

//执行一些命令的回显
void MainWindow::cmd_output()
{
    ui.cmd_output->append(quick_cmd->readAllStandardOutput());
}

//执行一些命令的错误回显
void MainWindow::cmd_error_output()
{
    ui.cmd_output->append("<font color=\"#FF0000\">"+quick_cmd->readAllStandardError()+"</font> ");
}

//析构函数
MainWindow::~MainWindow()
{
    if( base_cmd)
    {
        delete base_cmd;
        base_cmd=NULL;
    }
    if(map_rviz)
    {
        delete map_rviz;
        map_rviz=NULL;
    }
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */
void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
//    ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    //QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "novauto_monitor");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui.line_edit_master->setEnabled(false);
        ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }

    QSettings return_pos("return-position","novauto_monitor");

    //读取快捷指令的setting
    QSettings quick_setting("quick_setting","novauto_monitor");
    QStringList ch_key=quick_setting.childKeys();
    for(auto c:ch_key)
    {
        add_quick_cmd(c,quick_setting.value(c,QString("")).toString());
    }

}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "novauto_monitor");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    //settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

    //存下快捷指令的setting
    QSettings quick_setting("quick_setting","novauto_monitor");
    quick_setting.clear();
    for(int i=0;i<ui.treeWidget_quick_cmd->topLevelItemCount();i++)
    {
        QTreeWidgetItem *top=ui.treeWidget_quick_cmd->topLevelItem(i);
        QTextEdit *cmd_val=(QTextEdit *)ui.treeWidget_quick_cmd->itemWidget(top->child(0),1);
        quick_setting.setValue(top->text(0),cmd_val->toPlainText());
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace cyrobot_monitor



