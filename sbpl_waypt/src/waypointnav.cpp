#include <waypointnav.hpp>

using namespace std;
using namespace cv;
//max define
ofstream path;
//ifstream readpath;


 void sbplWaypointNav::botpos_sub(const nav_msgs::Odometry botpos){
  bot_pos = botpos;
   
  tf::Quaternion q(botpos.pose.pose.orientation.x, botpos.pose.pose.orientation.y, botpos.pose.pose.orientation.z, botpos.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    bot_yaw=yaw;
}

void sbplWaypointNav::proptarget_sub(const geometry_msgs::PoseStamped msgtarget){
  target_pos = msgtarget;
  /*tf::Quaternion q(msgtarget.pose.orientation.x, msgtarget.pose.orientation.y, msgtarget.pose.orientation.z, msgtarget.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw,target_yaw;
  m.getRPY(roll, pitch, yaw);
  target_yaw=yaw;*/
}

void sbplWaypointNav::laserscan_sub(const sensor_msgs::LaserScan msg){
  scandata = msg;
}

void sbplWaypointNav::printdata(){
  //Mat img(mapsize,mapsize,CV_8UC3,Scalar(0,0,0));
  path.open("my_env1.cfg");
  string s[10];
  s[0]="discretization(cells): 2240 2240";
  s[1]="obsthresh: 1";
  s[2]="cost_inscribed_thresh: 1";
  s[3]="cost_possibly_circumscribed_thresh: 0"; 
  s[4]="cellsize(meters): 0.025";
  s[5]="nominalvel(mpersecs): 1.0";
  s[6]="timetoturn45degsinplace(secs): 2.0";
  s[7]="start(meters,rads): ";
  s[8]="end(meters,rads): ";
  s[9]="environment:";
  //cout<<"bot pose:"<<bot_pos.pose.pose.position.x <<" "<<bot_pos.pose.pose.position.y<<" "<<bot_yaw<<endl;
  //cout<<"transform: "<<target_base_link.pose.position.x<<" "<<target_base_link.pose.position.y<<" "<<base_link_yaw<<endl;
  //cout<<"target pos:"<<target_pos.pose.position.x<<" "<<target_pos.pose.position.y<<" "<<endl;

  path<<s[0]<<std::endl<<s[1]<<std::endl<<s[2]<<std::endl<<s[3]<<std::endl<<s[4]<<std::endl<<s[5]<<std::endl<<s[6]<<std::endl<<s[7]<<28-bot_pos.pose.pose.position.y<<" "<<14+bot_pos.pose.pose.position.x<<" "<<bot_yaw<<std::endl<<s[8]<<28-target_pos.pose.position.y<<" "<<14+target_pos.pose.position.x<<" "<<target_yaw<<std::endl<<s[9]<<std::endl;
  path.close();
}

void sbplWaypointNav::update_map(){

        cout<<"BOT_POS_X: "<<(int)((bot_pos.pose.pose.position.x/56.00)*mapsize)<<"  BOT_POS_Y: "<<(int)((bot_pos.pose.pose.position.y/56.00)*mapsize)<<"\n";                  
               int bot_pix_x=((0.75*mapsize)-1)-( (bot_pos.pose.pose.position.x/56.00)*mapsize);
               int bot_pix_y=((mapsize/2)-1)-( (bot_pos.pose.pose.position.y/56.00)*mapsize);
               cout<<"bot_pix_x: "<<bot_pix_x<<" bot_pix_y: "<<bot_pix_y<<endl;
      //path.open("my_env.cfg",ios_base::app);  
      /*convert laserscan data to point cloud.ref:http://wiki.ros.org/laser_geometry*/
        Mat img(mapsize,mapsize,CV_8UC3,Scalar(0,0,0));
        img=img-img;
        int i=0;
        laser_geometry::LaserProjection projector_;
         tf::StampedTransform transform;

        //tf::TransformListener listener_;
        /*if(!listener_.waitForTransform(scandata.header.frame_id,"/base_link",scandata.header.stamp + 
          ros::Duration().fromSec(scandata.ranges.size()*scandata.time_increment),ros::Duration(1.0)))
          {return;}*/
              sensor_msgs::PointCloud cloud;//the points in cloud are (metre,metre) relative to center


              projector_.projectLaser(scandata, cloud);
             //projector_.transformLaserScanToPointCloud("/base_link",scandata,cloud,listener_);
                


              /*transform the point cloud into odom frame*/
              long long int Numcldpts=cloud.points.size();
              for(int i =0;i<Numcldpts;i++){
                 geometry_msgs::PoseStamped pt;
                 geometry_msgs::PoseStamped pt_transformed;
                 pt.header = cloud.header;

                 //shifting coordinate to (i,j)->(42,28)
                 pt.pose.position.x = cloud.points[i].x;
                 pt.pose.position.y = cloud.points[i].y;
//                 ROS_ERROR("----");


                 //cout<<pt.pose.position.x<<"___"<<pt.pose.position.y<<endl;


                 try {
                listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0) );
                listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
                  } catch (tf::TransformException ex) {
                  ROS_ERROR("%s",ex.what());
                  }
                  tf::Vector3 vector_base_link = tf::Vector3(pt.pose.position.x, pt.pose.position.y, 0);
                  tf::Vector3 vector_odom = transform(vector_base_link);
                  pt_transformed.pose.position.x = vector_odom.getX();
                  pt_transformed.pose.position.y = vector_odom.getY();
                            
                 /*try{
                    listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0) );
                    listener.transformPose("/odom", pt, pt_transformed);
                  } catch (tf::TransformException ex) {
                      ROS_ERROR("%s",ex.what());
                    }
                 //listener.transformPoint("/odom", pt, pt_transformed);
                    //print it*/
                 //cout<<pt_transformed.pose.position.x<<" "<<pt_transformed.pose.position.y<<endl;
                 //update in glob map
                   
                  int glob_x=(0.75*mapsize)-( (pt_transformed.pose.position.x/56.00)*mapsize);
                  int glob_y=(0.5*mapsize)-( (pt_transformed.pose.position.y/56.00)*mapsize);
                 //cout<<glob_y<<"  "<<glob_x<<endl;
                  if(glob_x>0&&glob_x<2240&&glob_y>0&&glob_y<2240)
                    glob_map[ glob_x][glob_y]=1;
                 //write to config file
                 ///doubt for infff 

              }

              for(int i=0;i<mapsize;i++)
              {
                for(int j=0;j<mapsize;j++)
                {
                  if(glob_map[i][j]==1)
                  {
                  img.at<Vec3b>(i,j)[0]=0;
                  img.at<Vec3b>(i,j)[1]=0;
                  img.at<Vec3b>(i,j)[2]=255;
                  }
                }
              }
              //target position in pixels
               cout<<"TARGET_POS_X: "<<(target_pos.pose.position.x/56.00)*mapsize<<"  TARGET_POS_Y: "<<(target_pos.pose.position.y/56.00)*mapsize<<endl;
               int target_pix_x=((0.75*mapsize)-1)-( (target_pos.pose.position.x/56.00)*mapsize);
               int target_pix_y=((mapsize/2)-1)-( (target_pos.pose.position.y/56.00)*mapsize);
               cout<<"target_pix_x: "<<target_pix_x<<" target_pix_y: "<<target_pix_y<<endl;//target_pix_x gives rows and target_pix_y gives columns
               



               cv::circle(img, cv::Point(target_pix_y,target_pix_x), 50, CV_RGB(255,0,0));//convention is just opposite as in i->j and j->i
                cv::circle(img, cv::Point(bot_pix_y, bot_pix_x), 50, CV_RGB(0,255,0));

              Size size(560,560);
              resize(img,img,size);
              imshow("odom_map visual",img);
              waitKey(5);

}

/*void sbplWaypointNav::map_zero(){
  static int i=0;
  i++;
  if(i>1){
    return;
  }
  path.open("my_env.cfg", ios_base::app);
  for(int m=0;m<mapsize;m++){
    for(int n=0;n<mapsize;n++){
      path<<"0 ";
    }
    path<<endl;
  }
}*/

void sbplWaypointNav::create_costmap(){
  path.open("my_env1.cfg",ios_base::app);
      int i,j;
      for(i=0;i<mapsize;i++){
         for(j=0;j<mapsize;j++){
            path<<glob_map[2239-i][j]<<" ";
         }
         path<<std::endl;
      }
      path.close();



}

sbplWaypointNav::sbplWaypointNav(ros::NodeHandle &node_handle){
    odom_sub = node_handle.subscribe("/odometry/filtered", buffer_size, &sbplWaypointNav::botpos_sub, this);
    target_sub = node_handle.subscribe("/waypoint_navigator/proposed_target", buffer_size, &sbplWaypointNav::proptarget_sub, this);
    scan_sub = node_handle.subscribe("/scan", buffer_size, &sbplWaypointNav::laserscan_sub, this);
    //lane_sub = node_handle.subscribe("/cloud_data", buffer_size, &sbplWaypointNav::pointcloud, this);
}