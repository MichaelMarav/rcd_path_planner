#include "ros/ros.h"
#include "apf_planner/apf.h"

int main(int argc, char **argv){

    ros::init(argc,argv,"apf_path_planning");
    ros::NodeHandle nh;
    apf_planner test(nh);


}
