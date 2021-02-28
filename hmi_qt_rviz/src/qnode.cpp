/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/cyrobot_monitor/qnode.hpp"
#include <QDebug>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{
    //读取topic的设置
    QSettings topic_setting("topic_setting","novauto_monitor");
    power_topic=topic_setting.value("topic_power","power").toString();
    power_min=topic_setting.value("power_min","10").toString();
    power_max=topic_setting.value("power_max","12").toString();
}

QNode::~QNode()
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc,init_argv,"novauto_monitor");
    if ( ! ros::master::check() )
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    // Add your ros communications here.
    n_ = new ros::NodeHandle();
    //0. camera
    image_sub0=n_ -> subscribe(camera_0_topic.toStdString(),100,&QNode::compressedImageCallback,this);
    //1. traffic signal light status
    pub_traffic_light = n_ -> advertise<zzz_perception_msgs::DetectionBoxArray>("/zzz/perception/traffic_lights", 10);
    //2. gps location
    sub_gps_fix=n_ -> subscribe(gps_topic.toStdString(),100, &QNode::gpsFixCallback,this);
    //3. gps velocity
    sub_gps_velocity=n_ -> subscribe(gps_velocity_topic.toStdString(),100, &QNode::gpsVelocityCallback,this);
    //4. imu pitch yaw roll
    sub_imu_data = n_ -> subscribe(imu_topic.toStdString(),100, &QNode::ImuDataCallback,this);
    //5. gear status
    sub_gear_sub = n_ -> subscribe(gear_status_topic.toStdString(), 100, &QNode::gearStatusCallback, this);
    //6. battery
    power_sub=n_ -> subscribe(power_topic.toStdString(),1000, &QNode::powerCallback,this);
    //7. sensor checker 09.07
//    pub_sensor_status = n.advertise<sensor_status_msgs::SensorStatus>("/sensor_status", 10);
    sub_sensor_status = n_ -> subscribe("/sensor_status", 10, &QNode::sensorStatusCallback, this);
    //8. can
    sub_acc_sub = n_ -> subscribe(can_status_topic.toStdString(),100,&QNode::canStatusCallback,this);
    //9.wheel degree
    sub_wheel_deg = n_ -> subscribe(wheel_deg_topic.toStdString(),100,&QNode::wheelDegCallback,this);
    start();
    return true;
}

//初始化的函数*********************************
bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"novauto_monitor");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    n_ = new ros::NodeHandle();
    //0. camera
    image_sub0=n_ -> subscribe(camera_0_topic.toStdString(),100,&QNode::compressedImageCallback,this);
    //1. traffic signal light status
    pub_traffic_light = n_ -> advertise<zzz_perception_msgs::DetectionBoxArray>("/zzz/perception/traffic_lights", 10);
    //2. gps location
    sub_gps_fix=n_ -> subscribe(gps_topic.toStdString(),100, &QNode::gpsFixCallback,this);
    //3. gps velocity
    sub_gps_velocity=n_ -> subscribe(gps_velocity_topic.toStdString(),100, &QNode::gpsVelocityCallback,this);
    //4. imu pitch yaw roll
    sub_imu_data = n_ -> subscribe(imu_topic.toStdString(),100, &QNode::ImuDataCallback,this);
    //5. gear status
    sub_gear_sub = n_ -> subscribe(gear_status_topic.toStdString(), 100, &QNode::gearStatusCallback, this);
    //6. battery
    power_sub=n_ -> subscribe(power_topic.toStdString(),1000, &QNode::powerCallback,this);
    //7. sensor checker 09.07
//    pub_sensor_status = n.advertise<sensor_status_msgs::SensorStatus>("/sensor_status", 10);
    sub_sensor_status = n_ -> subscribe("/sensor_status", 10, &QNode::sensorStatusCallback, this);
    //8. can
    sub_acc_sub = n_ -> subscribe(can_status_topic.toStdString(),100,&QNode::canStatusCallback,this);
    //9.wheel degree
    sub_wheel_deg = n_ -> subscribe(wheel_deg_topic.toStdString(),100,&QNode::wheelDegCallback,this);
    start();
    return true;
}

QMap<QString,QString> QNode::get_topic_list()
{
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);
    QMap<QString,QString> res;
    for(auto topic:topic_list)
    {
        res.insert(QString::fromStdString(topic.name),QString::fromStdString(topic.datatype));
    }
    return res;
}

void QNode::run()
{
    int count=0;
    ros::Rate loop_rate(1);
    //当当前节点没有关闭时
    while ( ros::ok() )
    {
        //调用消息处理回调函数
        ros::spinOnce();
        loop_rate.sleep();
    }
    //如果当前节点关闭
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

//sub gps
void QNode::Sub_Gps(QString topic)
{
    gps_topic = topic;
    sub_gps_fix = n_ -> subscribe(gps_topic.toStdString(),100,&QNode::gpsFixCallback,this);
}

void QNode::gpsFixCallback(const sensor_msgs::NavSatFixConstPtr &Gps_msg)
{
    emit Show_gps(Gps_msg -> longitude, Gps_msg -> latitude);
}

//sub velocity
void QNode::Sub_Velocity(QString topic)
{
    gps_velocity_topic = topic;
    sub_gps_velocity = n_ -> subscribe(gps_velocity_topic.toStdString(),100,&QNode::gpsVelocityCallback,this);
}

void QNode::gpsVelocityCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg)
{
    emit Show_velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
}

//sub imu data
void QNode::Sub_Imu(QString topic)
{
    imu_topic = topic;
    sub_imu_data = n_ -> subscribe(imu_topic.toStdString(),100,&QNode::ImuDataCallback,this);
}

void QNode::ImuDataCallback(const sensor_msgs::ImuPtr &Imu_msg)
{
    emit Show_imu_data(Imu_msg->orientation.x, Imu_msg->orientation.y, Imu_msg->orientation.z, Imu_msg->orientation.w);
}

//sub battery
void QNode::Sub_Power(QString topic)
{
    power_topic = topic;
    power_sub = n_ -> subscribe(power_topic.toStdString(),1000,&QNode::powerCallback,this);
}

void QNode::powerCallback(const canbus_msgs::BMSFbkConstPtr &Power_msg)
{
    emit power(Power_msg->SOC);
}

//sub acc
void QNode::Sub_Acc_Brk_Ped(QString topic)
{
    can_status_topic = topic;
    sub_acc_sub = n_ -> subscribe(can_status_topic.toStdString(),100,&QNode::canStatusCallback,this);
}

void QNode::canStatusCallback(const canbus_msgs::SCU_IPC_6_0x209ConstPtr &can_msg)
{
//    qDebug()<<can_msg->SCU_IPC_AccPedalSig <<"   "<<can_msg->SCU_IPC_BrkPedalSt;
    emit acc_pedal(can_msg->SCU_IPC_AccPedalSig);
    emit brake_pedal(can_msg->SCU_IPC_BrkPedalSt);
    emit power(can_msg->SCU_IPC_dstBat_Dsp * 100 / 360);
    emit Show_gear_status(can_msg->SCU_IPC_CurrentGearLev);
}

void QNode::Sub_wheel_Deg(QString topic)
{
    wheel_deg_topic = topic;
    sub_wheel_deg = n_ -> subscribe(wheel_deg_topic.toStdString(),100,&QNode::wheelDegCallback,this);
}

void QNode::wheelDegCallback(const canbus_msgs::SCU_IPC_1_0x204ConstPtr &can_msg)
{
    emit wheel_degree(can_msg->SCU_IPC_SteeringAngle);
}

//sub gear status
void QNode::Sub_Gear_Status(QString topic)
{
    gear_status_topic = topic;
    sub_gear_sub = n_ -> subscribe(gear_status_topic.toStdString(), 100, &QNode::gearStatusCallback, this);
}

void QNode::gearStatusCallback(const xpmotors_can_msgs::AutoStateConstPtr &Gear_msg)
{
    emit Show_gear_status(Gear_msg->GearState);
}

//订阅图片话题，并在label上显示
void QNode::Sub_Image(QString topic,int frame_id)
{
    camera_0_topic = topic;
//    image_transport::ImageTransport it_(n_);
    switch (frame_id) {
    case 0:
        //            image_sub0=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback0,this);
        image_sub0=n_ -> subscribe(topic.toStdString(),100,&QNode::compressedImageCallback,this);
        break;
    case 1:
        //             image_sub1=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback1,this);
        image_sub1=n_ -> subscribe(topic.toStdString(),100,&QNode::imageCallback1,this);
        break;
    case 2:
        //             image_sub2=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback2,this);
        image_sub2=n_ -> subscribe(topic.toStdString(),100,&QNode::imageCallback2,this);
        break;
    case 3:
        //             image_sub3=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback3,this);
        image_sub3=n_ -> subscribe(topic.toStdString(),100,&QNode::imageCallback3,this);
        break;
    }
    ros::spinOnce();
}

//图像话题的回调函数
void QNode::imageCallback0(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        //深拷贝转换为opencv类型
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        QImage im=Mat2QImage(cv_ptr->image);
        emit Show_image(0,im);
    }
    catch (cv_bridge::Exception& e)
    {

        log(Error,("video frame0 exception: "+QString(e.what())).toStdString());
        return;
    }
}

void QNode::compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImage cv_image;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.header.stamp = msg->header.stamp;
        // convert compressed image data to cv::Mat
        cv_image.image = cv::imdecode(cv::Mat(msg->data),1);
        QImage im=Mat2QImage(cv_image.image);
        emit Show_image(0,im);
    }
    catch (cv_bridge::Exception& e)
    {
        log(Error,("video frame0 exception: "+QString(e.what())).toStdString());
        return;
    }
}

//图像话题的回调函数
void QNode::imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //深拷贝转换为opencv类型
        cv_ptr = cv_bridge::toCvCopy(msg,video1_format.toStdString());
        QImage im=Mat2QImage(cv_ptr->image);
        emit Show_image(1,im);
    }
    catch (cv_bridge::Exception& e)
    {
        log(Error,("video frame1 exception: "+QString(e.what())).toStdString());
        return;
    }
}
//图像话题的回调函数
void QNode::imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //深拷贝转换为opencv类型
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        QImage im=Mat2QImage(cv_ptr->image);
        emit Show_image(2,im);
    }
    catch (cv_bridge::Exception& e)
    {
        log(Error,("video frame2 exception: "+QString(e.what())).toStdString());
        return;
    }
}
//图像话题的回调函数
void QNode::imageCallback3(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //深拷贝转换为opencv类型
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        QImage im=Mat2QImage(cv_ptr->image);
        emit Show_image(3,im);
    }
    catch (cv_bridge::Exception& e)
    {
        log(Error,("video frame3 exception: "+QString(e.what())).toStdString());
        return;
    }
}

//sensor checker callback 09.07
void QNode::sensorStatusCallback(const sensor_status_msgs::SensorStatusConstPtr &msg)
{
    lidar_status_ok = msg -> lidar_status_ok;
    canbus_status_ok = msg -> canbus_status_ok;
    gnss_status_ok = msg -> gnss_status_ok;
    camera_status_ok = msg -> camera_status_ok;
    arsmmw_status_ok = msg -> arsmmw_status_ok;
    srrmmw_status_ok = msg -> srrmmw_status_ok;
    usradar_status_ok = msg -> usradar_status_ok;
    localization_status_ok = msg -> localization_status_ok;
    perception_status_ok = msg -> perception_status_ok;
    control_status_ok = msg -> control_status_ok;
    planning_status_ok = msg -> planning_status_ok;
    emit sensor_self_check(lidar_status_ok, canbus_status_ok,\
                           gnss_status_ok, camera_status_ok,\
                           localization_status_ok, perception_status_ok,\
                           control_status_ok, planning_status_ok);
}

// QImage QNode::Mat2QImage(cv::Mat const& src)
// {
//   QImage dest(src.cols, src.rows, QImage::Format_ARGB32);

//   const float scale = 255.0;

//   if (src.depth() == CV_8U) {
//     if (src.channels() == 1) {
//       for (int i = 0; i < src.rows; ++i) {
//         for (int j = 0; j < src.cols; ++j) {
//           int level = src.at<quint8>(i, j);
//           dest.setPixel(j, i, qRgb(level, level, level));
//         }
//       }
//     } else if (src.channels() == 3) {
//       for (int i = 0; i < src.rows; ++i) {
//         for (int j = 0; j < src.cols; ++j) {
//           cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
//           dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
//         }
//       }
//     }
//   } else if (src.depth() == CV_32F) {
//     if (src.channels() == 1) {
//       for (int i = 0; i < src.rows; ++i) {
//         for (int j = 0; j < src.cols; ++j) {
//           int level = scale * src.at<float>(i, j);
//           dest.setPixel(j, i, qRgb(level, level, level));
//         }
//       }
//     } else if (src.channels() == 3) {
//       for (int i = 0; i < src.rows; ++i) {
//         for (int j = 0; j < src.cols; ++j) {
//           cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
//           dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
//         }
//       }
//     }
//   }

//   return dest;
// }

QImage QNode::Mat2QImage(cv::Mat const& mat)
{
    cvtColor(mat, mat, cv::COLOR_BGR2RGB);
    QImage qim((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step,
               QImage::Format_RGB888);
    return qim;
}

void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace cyrobot_monitor
