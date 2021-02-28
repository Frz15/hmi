/**
 * @file /include/cyrobot_monitor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cyrobot_monitor_QNODE_HPP_
#define cyrobot_monitor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QLabel>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h> // for topic /localization/gps/vel
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>              //cv_bridge
#include <sensor_msgs/image_encodings.h>    //图像编码格式
#include <sensor_msgs/NavSatFix.h>  //gps
#include <sensor_msgs/Imu.h>        //IMU
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui/highgui.hpp>
#include <map>
#include <QLabel>
#include <QImage>
#include <QSettings>
//zzz perception msgs
#include <zzz_perception_msgs/DetectionBoxArray.h>
#include <sensor_status_msgs/SensorStatus.h>
#include <canbus_msgs/BMSFbk.h>
#include <xpmotors_can_msgs/AutoState.h>
#include <canbus_msgs/SCU_IPC_6_0x209.h>
#include <canbus_msgs/SCU_IPC_1_0x204.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void Sub_Image(QString topic,int frame_id = 0);
    QMap<QString,QString> get_topic_list();
    void run();

    /*********************
        ** Logging
        **********************/
    enum LogLevel {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg);

    //GPS
    void Sub_Gps(QString topic);
    //velocity
    void Sub_Velocity(QString topic);
    //imu data
    void Sub_Imu(QString topic);
    //test car battery
    void Sub_Power(QString topic);
    //accelerator, brake pedal
    void Sub_Acc_Brk_Ped(QString topic);
    //wheel degree
    void Sub_wheel_Deg(QString topic);
    //can gear status
    void Sub_Gear_Status(QString topic);
    //publish traffic signal light status
    ros::Publisher pub_traffic_light;

    //sensor_checker 09.05
//    ros::Publisher pub_sensor_status;

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    //test_car battery
    void power(int p);
    void acc_pedal(int p);
    void brake_pedal(int p);
    void wheel_degree(float p);
    void Master_shutdown();
    void Show_image(int,QImage);

    //GPS
    void Show_gps(double lon, double lat);

    //Velocity
    void Show_velocity(double vel_x, double vel_y);

    //IMU data
    void Show_imu_data(double ori_x, double ori_y, double ori_z, double ori_w);

    //sensor checker
    void sensor_self_check(int, int, int, int,\
                           int, int, int, int);
    //can gear status
    void Show_gear_status(int);

private:
    ros::NodeHandle *n_;
    int init_argc;
    char** init_argv;
    QStringListModel logging_model;
    //图像订阅
    ros::Subscriber image_sub0;
    QString camera_0_topic = "/left_usb_cam/image_raw/compressed";
    void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg);

    ros::Subscriber image_sub1;
    ros::Subscriber image_sub2;
    ros::Subscriber image_sub3;

    //gps topic
    ros::Subscriber sub_gps_fix;
    QString gps_topic = "/localization/gps/fix";
    void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr &Gps_msg);

    //velocity topic
    ros::Subscriber sub_gps_velocity;
    QString gps_velocity_topic = "/localization/gps/vel";
    void gpsVelocityCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg);

    //IMU data topic
    ros::Subscriber sub_imu_data;
    QString imu_topic = "/localization/imu/data";
    void ImuDataCallback(const sensor_msgs::ImuPtr &Imu_msg);

    //test_car battery
    ros::Subscriber power_sub;
    QString power_topic = "/CAN/BMS_fbk";
    void powerCallback(const canbus_msgs::BMSFbkConstPtr &Power_msg);

    //can gear status
    ros::Subscriber sub_gear_sub;
    QString gear_status_topic = "/xp/auto_state";
    void gearStatusCallback(const xpmotors_can_msgs::AutoStateConstPtr &Gear_msg);

    //can accelerator pedal, brake pedal status
    ros::Subscriber sub_acc_sub;
    QString can_status_topic = "/canbus/SCU_IPC_6";
    void canStatusCallback(const canbus_msgs::SCU_IPC_6_0x209ConstPtr &can_msg);

    //can wheel degree
    ros::Subscriber sub_wheel_deg;
    QString wheel_deg_topic = "/canbus/SCU_IPC_1";
    void wheelDegCallback(const canbus_msgs::SCU_IPC_1_0x204ConstPtr &can_msg);

    //图像format
    QString video0_format;
    QString video1_format;
    QString video2_format;
    QString video3_format;

    QString power_max;
    QString power_min;
    QImage Mat2QImage(cv::Mat const& src);


    void imageCallback0(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback3(const sensor_msgs::ImageConstPtr& msg);

    //sensor_checker 09.05
    ros::Subscriber sub_sensor_status;
    void sensorStatusCallback(const sensor_status_msgs::SensorStatusConstPtr& msg);
    int lidar_status_ok = 0;
    int canbus_status_ok = 0;
    int gnss_status_ok = 0;
    int camera_status_ok = 0;
    int arsmmw_status_ok = 0;
    int srrmmw_status_ok = 0;
    int usradar_status_ok = 0;
    int localization_status_ok = 0;
    int perception_status_ok = 0;
    int control_status_ok = 0;
    int planning_status_ok = 0;
};

}  // namespace cyrobot_monitor

#endif /* cyrobot_monitor_QNODE_HPP_ */
