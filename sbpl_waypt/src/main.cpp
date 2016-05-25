#include <waypointnav.hpp>

int main(int argc, char* argv[]) {

    std::string node_name = "sbpl_waypt"; 
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;
    sbplWaypointNav *sbpl_waypoint = new sbplWaypointNav(node_handle);
    ros::Rate loop_rate(10);
    while (ros::ok()){
    	
    	ros::spinOnce();
    	
    	sbpl_waypoint->printdata();
    	sbpl_waypoint->update_map();
    	sbpl_waypoint->create_costmap();
        system("../sbpl/build/test_sbpl  /home/rishabh/catkin_ws/my_env1.cfg /home/rishabh/sbpl/matlab/mprim/my_mprimitives.mprim");
        system(" python ../sbpl/build/vis.py /home/rishabh/catkin_ws/my_env1.cfg  sol.txt");

    	loop_rate.sleep();
    }
    //ros::spin();
    return 0;
}
