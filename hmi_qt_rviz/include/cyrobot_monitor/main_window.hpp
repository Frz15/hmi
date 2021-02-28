/**
 * @file /include/cyrobot_monitor/main_window.hpp
 *
 * @brief Qt based gui for cyrobot_monitor.
 *
 * @date November 2010
 **/
#ifndef cyrobot_monitor_MAIN_WINDOW_H
#define cyrobot_monitor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "addtopics.h"
#include "settings.h"
#include "qrviz.hpp"
//仪表盘头文件
#include "CCtrlDashBoard.h"
#include "QProcess"
#include <QStandardItemModel>
#include <QTreeWidgetItem>
#include <QComboBox>
#include <QSpinBox>
#include <QVariant>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <map>
//rviz
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool.h>
//gps location
#include "mapwidget.h"
//yaw
#include "steering_wheel.h"
#include "yaw_compass.h"
/*****************************************************************************
** Namespace
*****************************************************************************/
namespace cyrobot_monitor {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void initRviz();
    void initUis();
    void initVideos();
    void initTopicList();

public slots:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check );
    void on_button_disconnect_clicked(bool check );
    void on_checkbox_use_environment_stateChanged(int state);
    void slot_rosShutdown();
    void quick_cmds_check_change(int);
    void cmd_output();
    void cmd_error_output();
    void refreashTopicList();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_tab_manage_currentChanged(int);
    void slot_tab_Widget_currentChanged(int);
    void slot_add_topic_btn();
    void slot_choose_topic(QTreeWidgetItem *choose);
    void slot_treewidget_item_value_change(QString);
    void slot_treewidget_item_check_change(int);
    //设置界面
    void slot_setting_frame();

    void quick_cmd_add();
    void quick_cmd_remove();
    //显示图像
    void slot_show_image(int,QImage);

    //camera
    void slot_camera_0_topic_value_change(QString);
    //battery
    void slot_power(int p);
    void slot_battery_topic_value_change(QString);
    //acc
    void slot_acc(int p);
    //brake
    void slot_brake(int p);
    void slot_pedal_topic_value_change(QString);
    //wheel
    void slot_wheel_degree(float p);
    void slot_wheel_topic_value_change(QString);
    //gps
    void slot_show_gps(double lon, double lat);
    void slot_localization_topic_value_change(QString);
    //velocity
    void slot_show_velocity(double vel_x, double vel_y);
    void slot_velocity_topic_value_change(QString);
    //imu data
    void slot_show_imu_data(double x, double y, double z, double w);
    void slot_imu_yaw_topic_value_change(QString);
    //fix frame name
    void slot_fixed_frame_value_change(QString);
    //pointcloud name
    void slot_pc_topic_value_change(QString);
    //pointcloud markerarray name
    void slot_pc_markerarray_topic_value_change(QString);
    void slot_lrr_markerarray_topic_value_change(QString value);
    //decision trajectory path
    void slot_decision_trajectory_path_topic_value_change(QString);
    //red light signal
    void on_red_signal_clicked(bool check );
    //green light signal
    void on_green_signal_clicked(bool check);
    void red_sender();
    void green_sender();
    //can gear status
    void slot_show_gear_status(int gear_status);
    void slot_gear_topic_value_change(QString);
    //sensor self check status
    void slot_show_sensor_status(int,int,int,int,int,int,int,int);

private slots:
    void on_settingButton_toggled(bool state);

private:
	Ui::MainWindowDesign ui;
    void connections();
    void add_quick_cmd(QString name,QString shell);
    QNode qnode;
    CCtrlDashBoard *m_DashBoard_x;
    CCtrlDashBoard *m_DashBoard_y;
    YawCompass *steering_wheel;
    SteeringWheel *wheel_;
    QProcess *quick_cmd=NULL;
    QProcess *close_remote_cmd=NULL;
    QProcess *base_cmd=NULL;
    QRviz *map_rviz=NULL;
    QStandardItemModel* treeView_rviz_model=NULL;
    AddTopics *addtopic_form=NULL;
    //存放rviz treewidget当前显示的控件及控件的父亲的地址
    QMap <QWidget*,QTreeWidgetItem *> widget_to_parentItem_map;
    //存放rviz treewidget当前显示的控件及控件的父亲的地址
    QMap <QWidget*,QTreeWidgetItem *> widget_to_parentItem_map_first_tab;
    //存放状态栏的对应关系 display名 状态item
    QMap <QString,QTreeWidgetItem *> tree_rviz_stues;
    //存放display的当前值 item名，参数名称和值
    QMap <QTreeWidgetItem*,QMap<QString,QString>> tree_rviz_values;
    Settings *set=NULL;

    MapWidget *map_widget;
    //sensor_status_msgs::SensorStatus SensorStatusmsg_;
    QString m_red_SheetStyle_, m_green_SheetStyle_;
};
}// namespace cyrobot_monitor

#endif // cyrobot_monitor_MAIN_WINDOW_H
