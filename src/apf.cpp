// Add local adding constant for the avoiding the local minima. (to avoid creating obstacle when there is none
#include <apf_planner/apf.h>
#include "ros/ros.h"


apf_planner::apf_planner(ros::NodeHandle nh){
    ROS_INFO("Created Object \n");

        nh.getParam("/analog_feedback_topic",);
}
