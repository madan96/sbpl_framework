#include <vector>
#include <fstream>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <stdlib.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <iosfwd>
#include <ios>
#include "laser_geometry.h"
#include <sensor_msgs/PointCloud.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>


static const int buffer_size = 10;
#define rpos_x 1119
#define rpos_y 1119

#define mapsize 2240

class sbplWaypointNav{

    double target_yaw,bot_yaw;
    geometry_msgs::PoseStamped target_pos;
    nav_msgs::Odometry bot_pos;
    sensor_msgs::LaserScan scandata;

    tf::TransformListener listener_;
    tf::TransformListener listener;
    laser_geometry::LaserProjection projector_;
    ros::Subscriber odom_sub;
    ros::Subscriber target_sub;
    ros::Subscriber scan_sub;

public:

    int glob_map[mapsize][mapsize];
	void botpos_sub(const nav_msgs::Odometry botpos);
	void proptarget_sub(const geometry_msgs::PoseStamped msgtarget);
    void laserscan_sub(const sensor_msgs::LaserScan msg);
	void printdata();
    void update_map();
    void create_costmap();
    //void mapzero();
	
	sbplWaypointNav(ros::NodeHandle &node_handle);
};
