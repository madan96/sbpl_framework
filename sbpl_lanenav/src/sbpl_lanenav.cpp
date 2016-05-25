#include <sbpl_lanenav.hpp>

using namespace std;
using namespace cv;
ofstream path;



void sbplLaneNav::botpos_sub(const nav_msgs::Odometry botpos){
  bot_pos = botpos;

  tf::Quaternion q(botpos.pose.pose.orientation.x, botpos.pose.pose.orientation.y, botpos.pose.pose.orientation.z, botpos.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    bot_yaw=yaw;
}

void sbplLaneNav::proptarget_sub(const geometry_msgs::PoseStamped msgtarget){
  target_pos = msgtarget;
   
  tf::Quaternion q(msgtarget.pose.orientation.x, msgtarget.pose.orientation.y, msgtarget.pose.orientation.z, msgtarget.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    target_yaw=yaw;
}

void sbplLaneNav::laserscan_sub(const sensor_msgs::LaserScan msg){
  scandata = msg;
}

void sbplLaneNav::transform(){
    //Converts proposed target data from odom to base_link frame
    target_base_link.pose.position.x = target_pos.pose.position.x - bot_pos.pose.pose.position.x;
    target_base_link.pose.position.y = target_pos.pose.position.y - bot_pos.pose.pose.position.y;
    base_link_yaw = target_yaw - bot_yaw;
}

void sbplLaneNav::printdata(){
  Mat img(mapsize,mapsize,CV_8UC3,Scalar(0,0,0));
  path.open("my_env.cfg");
  string s[10];
  s[0]="discretization(cells): 2240 2240";
  s[1]="obsthresh: 1";
  s[2]="cost_inscribed_thresh: 1";
  s[3]="cost_possibly_circumscribed_thresh: 0";
  s[4]="cellsize(meters): 0.025";
  s[5]="nominalvel(mpersecs): 1.0";
  s[6]="timetoturn45degsinplace(secs): 2.0";
  s[7]="start(meters,rads): 28 28 0";
  s[8]="end(meters,rads): ";
  s[9]="environment:";
  cout<<"bot pose:"<<bot_pos.pose.pose.position.x <<" "<<bot_pos.pose.pose.position.y<<" "<<bot_yaw<<endl;
  cout<<"transform: "<<target_base_link.pose.position.x<<" "<<target_base_link.pose.position.y<<" "<<base_link_yaw<<endl;
  cout<<"target pos:"<<target_base_link.pose.position.x+bot_pos.pose.pose.position.x<<" "<<target_base_link.pose.position.y+bot_pos.pose.pose.position.y<<" "<<base_link_yaw<<endl;

 for(int i =0;i<img.rows;i++)
  {
    for(int j=0;j<img.cols;j++)
    {
      
      {
      img.at<Vec3b>(i,j)[0]=0;
      img.at<Vec3b>(i,j)[1]=0;
      img.at<Vec3b>(i,j)[2]=0;
    }
  }
}

     for(int i =0;i<img.rows;i++)
  {
    for(int j=0;j<img.cols;j++)
    {
      if(cost_map[i][j]==1)
    {
       img.at<Vec3b>(-500+2240-i,j)[0]=0;
      img.at<Vec3b>(-500+2240-i,j)[1]=255;
      img.at<Vec3b>(-500+2240-i,j)[2]=0;

    }
    }
  }

  //bot pose in pixel
int rel_x=((mapsize/2)-1)-( (target_base_link.pose.position.x/56.00)*mapsize);
int rel_y=((mapsize/2)-1)-( (target_base_link.pose.position.y/56.00)*mapsize);
cout<<rel_y<<" "<<rel_x<<endl;

  //--------
    cv::circle(img, cv::Point(rel_y, rel_x-500), 50, CV_RGB(255,255,255));
    cv::circle(img, cv::Point(mapsize/2, mapsize/2-500), 50, CV_RGB(255,255,255));


   
    /*for(int i=0;i<lane_msg.points.size();i++){
       
       int p_x=((mapsize/2)-1)-( (lane_msg.points[i].y/56.00)*mapsize);

        int p_y=((mapsize/2)-1)-( (lane_msg.points[i].x/56.00)*(mapsize));
       if(p_y<mapsize && p_y>0 && p_x<mapsize && p_x>0){
       cout<<p_x<<" "<<p_y<<endl;
       img.at<Vec3b>(p_y,p_x)[0]=0;
       img.at<Vec3b>(p_y,p_x)[1]=0;
       img.at<Vec3b>(p_y,p_x)[2]=255;
     }
    }*/
     // Size size(560,560);
       // resize(img,img,size);

  imshow("lane_navigator", img);
  waitKey(5);

  path<<s[0]<<std::endl<<s[1]<<std::endl<<s[2]<<std::endl<<s[3]<<std::endl<<s[4]<<std::endl<<s[5]<<std::endl<<s[6]<<std::endl<<s[7]<<std::endl<<s[8]<<-target_base_link.pose.position.y+28<<" "<<target_base_link.pose.position.x+28<<" "<<base_link_yaw<<std::endl<<s[9]<<std::endl;
  path.close();
}

void sbplLaneNav::create_costmap(){
  path.open("my_env.cfg",ios_base::app);
      int size = (scandata.angle_max - scandata.angle_min)/scandata.angle_increment;

      for(int i=0;i<2240;i++)
      {
        for(int j=0;j<2240;j++)
        {
          cost_map[i][j]=0;
        }
      }
      for(int m=0;m<size;m++){
         if(scandata.ranges[m]<scandata.range_max && scandata.ranges[m]>scandata.range_min)
         cost_map[rpos_y-(int)(40*scandata.ranges[m]*cos(scandata.angle_min + m*(scandata.angle_increment)))][rpos_x-(int)(40*scandata.ranges[m]*sin(scandata.angle_min + m*(scandata.angle_increment)))]=1;
         //cout<<"Angle min: "<<scandata.angle_min<<std::endl<<"angle_increment: "<<scandata.angle_increment<<std::endl<<"ranges: "<<scandata.ranges[m]<<std::endl;
      }
  
      for(int i =0;i<lane_msg.points.size();i++)
      {
        int p_x=((mapsize/2)-1)- ( (lane_msg.points[i].y/56.00)*(mapsize));//mapsize/2 see 

        int p_y=((mapsize/2)-1)+( (lane_msg.points[i].x/56.00)*(mapsize));
        cost_map[p_y][p_x]=1;
      }
      int i,j;
      for(i=0;i<mapsize;i++){
         for(j=0;j<mapsize;j++){
            path<<cost_map[i][j]<<" ";
         }
         path<<std::endl;
      }
      path.close();
}

void sbplLaneNav::pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
  
  lane_msg = *cloud;
}

sbplLaneNav::sbplLaneNav(ros::NodeHandle &node_handle){
    odom_sub = node_handle.subscribe("/odometry/filtered", buffer_size, &sbplLaneNav::botpos_sub, this);
    target_sub = node_handle.subscribe("/lane_navigator/proposed_target", buffer_size, &sbplLaneNav::proptarget_sub, this);
    scan_sub = node_handle.subscribe("/scan", buffer_size, &sbplLaneNav::laserscan_sub, this);
    lane_sub = node_handle.subscribe("/cloud_data", buffer_size, &sbplLaneNav::pointcloud, this);

        //transform(const nav_msgs::Odometry bot_pos, const geometry_msgs::PoseStamped target_pos, double bot_yaw, double target_yaw);
    //printdata();
}