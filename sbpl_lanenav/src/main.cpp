#include <sbpl_lanenav.hpp>

int main(int argc, char* argv[]) {

	std::string node_name = "sbpl_lanenav"; 
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;
    sbplLaneNav *sbpl_subscriber = new sbplLaneNav(node_handle);
    ros::Rate loop_rate(10);
    while (ros::ok()){
    	
    	ros::spinOnce();
    	sbpl_subscriber->transform();
    	sbpl_subscriber->printdata();
    	sbpl_subscriber->create_costmap();
        system("../sbpl/build/test_sbpl  /home/rishabh/catkin_ws/my_env.cfg /home/rishabh/sbpl/matlab/mprim/my_mprimitives.mprim");
        system(" python ../sbpl/build/vis.py /home/rishabh/catkin_ws/my_env.cfg  sol.txt");

    	loop_rate.sleep();
    }
    //ros::spin();
    return 0;
}
