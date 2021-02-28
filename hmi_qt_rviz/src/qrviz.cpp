#include "../include/cyrobot_monitor/qrviz.hpp"

QRviz::QRviz(QVBoxLayout *layout,QString node_name)
{
    this->layout=layout;
    this->nodename=node_name;

    //创建rviz容器
    render_panel_=new rviz::RenderPanel;
    //向layout添加widget
    layout->addWidget(render_panel_);
    //初始化rviz控制对象
    manager_=new rviz::VisualizationManager(render_panel_);
    ROS_ASSERT(manager_!=NULL);
    //获取当前rviz控制对象的 tool控制对象
    tool_manager_=manager_->getToolManager();
    ROS_ASSERT(tool_manager_!=NULL);
    //初始化camera 这行代码实现放大 缩小 平移等操作
    render_panel_->initialize(manager_->getSceneManager(),manager_);
    manager_->initialize();
    tool_manager_->initialize();
    manager_->removeAllDisplays();
}

rviz::Display* RobotModel_=NULL;
//显示robotModel
void QRviz::Display_RobotModel(bool enable)
{

    if(RobotModel_==NULL)
    {
        RobotModel_=manager_->createDisplay("rviz/RobotModel","Qrviz RobotModel",enable);
    }
    else{
        delete RobotModel_;
        RobotModel_=manager_->createDisplay("rviz/RobotModel","Qrviz RobotModel",enable);
    }
}

//显示grid
void QRviz::Display_Grid(bool enable,QString Reference_frame,int Plan_Cell_count,QColor color)
{
    if(grid_==NULL)
    {
        grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
        ROS_ASSERT( grid_ != NULL );
        // Configure the GridDisplay the way we like it.
        grid_->subProp( "Line Style" )->setValue("Billboards");
        grid_->subProp( "Color" )->setValue(color);
        grid_->subProp( "Reference Frame" )->setValue(Reference_frame);
        grid_->subProp("Plane Cell Count")->setValue(Plan_Cell_count);
    }
    else{
        delete grid_;
        grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
        ROS_ASSERT( grid_ != NULL );
        // Configure the GridDisplay the way we like it.
        grid_->subProp( "Line Style" )->setValue("Billboards");
        grid_->subProp( "Color" )->setValue(color);
        grid_->subProp( "Reference Frame" )->setValue(Reference_frame);
        grid_->subProp("Plane Cell Count")->setValue(Plan_Cell_count);
    }
    grid_->setEnabled(enable);
    manager_->startUpdate();
}

//显示map
void QRviz::Display_Map(bool enable,QString topic,double Alpha,QString Color_Scheme)
{
    if(!enable&&map_)
    {
        map_->setEnabled(false);
        return ;
    }
    if(map_==NULL)
    {
        map_=manager_->createDisplay("rviz/Map","QMap",true);
        ROS_ASSERT(map_);
        map_->subProp("Topic")->setValue(topic);
        map_->subProp("Alpha")->setValue(Alpha);
        map_->subProp("Color Scheme")->setValue(Color_Scheme);
    }
    else{
        ROS_ASSERT(map_);
        delete map_;
        map_=manager_->createDisplay("rviz/Map","QMap",true);
        ROS_ASSERT(map_);
        map_->subProp("Topic")->setValue(topic);
        map_->subProp("Alpha")->setValue(Alpha);
        map_->subProp("Color Scheme")->setValue(Color_Scheme);
    }

    map_->setEnabled(enable);
    manager_->startUpdate();
}

//显示激光雷达
void QRviz::Display_LaserScan(bool enable,QString topic)
{
    if(laser_==NULL)
    {
        laser_=manager_->createDisplay("rviz/LaserScan","QLaser",enable);
        ROS_ASSERT(laser_);
        laser_->subProp("Topic")->setValue(topic);
    }
    else{
        delete laser_;
        laser_=manager_->createDisplay("rviz/LaserScan","QLaser",enable);
        ROS_ASSERT(laser_);
        laser_->subProp("Topic")->setValue(topic);
    }
    qDebug()<<"topic:"<<topic;
    laser_->setEnabled(enable);
    manager_->startUpdate();
}

//显示激光雷达
void QRviz::Display_PointCloud2(bool enable,QString topic)
{
    QString class_id = "rviz/Orbit";
    ViewManager_ = manager_->getViewManager();
    ViewManager_->setCurrentViewControllerType(class_id);
    ViewManager_->getCurrent()->subProp("Distance")->setValue("110");
    ViewManager_->getCurrent()->subProp("Yaw")->setValue("3.1354");
    ViewManager_->getCurrent()->subProp("Pitch")->setValue("0.929797");
    ViewManager_->getCurrent()->subProp("Focal Point")->setValue("7;1;8");
    if(pointcloud2_==NULL)
    {
        pointcloud2_=manager_->createDisplay("rviz/PointCloud2","QPointCloud2",enable);
        ROS_ASSERT(pointcloud2_);
        pointcloud2_->subProp("Topic")->setValue(topic);
        pointcloud2_->subProp("Size (Pixels)")->setValue("2");
        pointcloud2_->subProp("Style")->setValue("Points");
        pointcloud2_->subProp("Queue Size")->setValue("10");
        pointcloud2_->subProp("Alpha")->setValue("0.6");
    }
    else{
        delete pointcloud2_;
        pointcloud2_=manager_->createDisplay("rviz/PointCloud2","QPointCloud2",enable);
        ROS_ASSERT(pointcloud2_);
        pointcloud2_->subProp("Topic")->setValue(topic);
        pointcloud2_->subProp("Size (Pixels)")->setValue("2");
        pointcloud2_->subProp("Style")->setValue("Points");
        pointcloud2_->subProp("Queue Size")->setValue("10");
        pointcloud2_->subProp("Alpha")->setValue("0.6");
    }
    pointcloud2_->setEnabled(enable);
    manager_->startUpdate();
}

void QRviz::Display_PointCloud2MarkerArray(bool enable,QString topic){
    if(pointcloud2MarkerArray_==NULL)
    {
        pointcloud2MarkerArray_=manager_->createDisplay("rviz/MarkerArray","QMarkerArray",enable);
        ROS_ASSERT(pointcloud2MarkerArray_);
        pointcloud2MarkerArray_->subProp("Marker Topic")->setValue(topic);
        pointcloud2MarkerArray_->subProp("Namespaces")->setValue("zzz/cognition");
        pointcloud2MarkerArray_->subProp("Queue Size")->setValue("100");
    }
    else{
        delete pointcloud2MarkerArray_;
        pointcloud2MarkerArray_=manager_->createDisplay("rviz/MarkerArray","QMarkerArray",enable);
        ROS_ASSERT(pointcloud2MarkerArray_);
        pointcloud2MarkerArray_->subProp("Marker Topic")->setValue(topic);
        pointcloud2MarkerArray_->subProp("Namespaces")->setValue("zzz/cognition");
        pointcloud2MarkerArray_->subProp("Queue Size")->setValue("100");
    }
    pointcloud2MarkerArray_->setEnabled(enable);
    manager_->startUpdate();
}

void QRviz::Display_radar2MarkerArray(bool enable,QString topic){
    if(radar2MarkerArray_==NULL)
    {
        radar2MarkerArray_=manager_->createDisplay("rviz/MarkerArray","QMarkerArray",enable);
        ROS_ASSERT(radar2MarkerArray_);
        radar2MarkerArray_->subProp("Marker Topic")->setValue(topic);
        radar2MarkerArray_->subProp("Namespaces")->setValue("zzz/cognition");
        radar2MarkerArray_->subProp("Queue Size")->setValue("100");
    }
    else{
        delete radar2MarkerArray_;
        radar2MarkerArray_=manager_->createDisplay("rviz/MarkerArray","QMarkerArray",enable);
        ROS_ASSERT(radar2MarkerArray_);
        radar2MarkerArray_->subProp("Marker Topic")->setValue(topic);
        radar2MarkerArray_->subProp("Namespaces")->setValue("zzz/cognition");
        radar2MarkerArray_->subProp("Queue Size")->setValue("100");
    }
    pointcloud2MarkerArray_->setEnabled(enable);
    manager_->startUpdate();
}

void QRviz::Display_DecisionTrajectoryPath(bool enable,QString topic)
{
    if(decisionTrajectoryPath_==NULL)
    {
        decisionTrajectoryPath_=manager_->createDisplay("rviz/Path","QPath",enable);
        ROS_ASSERT(decisionTrajectoryPath_);
        decisionTrajectoryPath_->subProp("Topic")->setValue(topic);
    }
    else{
        delete decisionTrajectoryPath_;
        decisionTrajectoryPath_=manager_->createDisplay("rviz/Path","QPath",enable);
        ROS_ASSERT(decisionTrajectoryPath_);
        decisionTrajectoryPath_->subProp("Topic")->setValue(topic);
    }
    decisionTrajectoryPath_->setEnabled(enable);
    manager_->startUpdate();
}

//设置全局显示
void QRviz::SetGlobalOptions(QString frame_name,QColor backColor,int frame_rate)
{
    manager_->setFixedFrame(frame_name);
    manager_->setProperty("Background Color",backColor);
    manager_->setProperty("Frame Rate",frame_rate);
    manager_->startUpdate();
}

// "rviz/MoveCamera";
// "rviz/Interact";
// "rviz/Select";
// "rviz/SetInitialPose";
// "rviz/SetGoal";
//设置机器人导航初始位置
void QRviz::Set_Pos()
{
    //获取设置Pos的工具
    //添加工具

    current_tool= tool_manager_->addTool("rviz/SetInitialPose");
    //设置当前使用的工具为SetInitialPose（实现在地图上标点）
    tool_manager_->setCurrentTool( current_tool );
    manager_->startUpdate();

    //     tool_manager_->setCurrentTool()

}
//设置机器人导航目标点
void QRviz::Set_Goal()
{
    //添加工具
    current_tool= tool_manager_->addTool("rviz/SetGoal");
    //设置goal的话题
    rviz::Property* pro= current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("/move_base_simple/goal");
    //设置当前frame
    manager_->setFixedFrame("map");
    //设置当前使用的工具为SetGoal（实现在地图上标点）
    tool_manager_->setCurrentTool( current_tool );

    manager_->startUpdate();

}
void QRviz::Set_MoveCamera()
{
    //获取设置Pos的工具
    //添加工具

    current_tool= tool_manager_->addTool("rviz/MoveCamera");
    //设置当前使用的工具为SetInitialPose（实现在地图上标点）
    tool_manager_->setCurrentTool( current_tool );
    manager_->startUpdate();
}
void QRviz::Set_Select()
{
    //获取设置Pos的工具
    //添加工具

    current_tool= tool_manager_->addTool("rviz/Select");
    //设置当前使用的工具为SetInitialPose（实现在地图上标点）
    tool_manager_->setCurrentTool( current_tool );
    manager_->startUpdate();
}
//显示tf坐标变换
void QRviz::Display_TF(bool enable)
{
    if(TF_){delete TF_;TF_=NULL;}
    TF_=manager_->createDisplay("rviz/TF","QTF",enable);
}
//显示导航相关
void QRviz::Display_Navigate(bool enable,QString Global_topic,QString Global_planner,QString Local_topic,QString Local_planner)
{
    if(Navigate_localmap) {delete Navigate_localmap; Navigate_localmap=NULL;}
    if(Navigate_localplanner) {delete Navigate_localplanner; Navigate_localplanner=NULL;}
    if(Navigate_globalmap) {delete Navigate_globalmap; Navigate_globalmap=NULL;}
    if(Navigate_globalplanner) {delete Navigate_globalplanner; Navigate_globalplanner=NULL;}
    //local map
    Navigate_localmap=manager_->createDisplay("rviz/Map","Qlocalmap",enable);
    Navigate_localmap->subProp("Topic")->setValue(Local_topic);
    Navigate_localmap->subProp("Color Scheme")->setValue("costmap");
    Navigate_localplanner=manager_->createDisplay("rviz/Path","QlocalPath",enable);
    Navigate_localplanner->subProp("Topic")->setValue(Local_planner);
    Navigate_localplanner->subProp("Color")->setValue(QColor(0,12,255));
    //global map
    Navigate_globalmap=manager_->createDisplay("rviz/Map","QGlobalmap",enable);
    Navigate_globalmap->subProp("Topic")->setValue(Global_topic);
    Navigate_globalmap->subProp("Color Scheme")->setValue("costmap");
    Navigate_globalplanner=manager_->createDisplay("rviz/Path","QGlobalpath",enable);
    Navigate_globalplanner->subProp("Topic")->setValue(Global_planner);
    Navigate_globalplanner->subProp("Color")->setValue(QColor(255,0,0));
    //更新画面显示
    manager_->startUpdate();

}
void QRviz::addTool( rviz::Tool* )
{

}
void QRviz::createDisplay(QString display_name,QString topic_name)
{


}
void QRviz::run()
{

}
