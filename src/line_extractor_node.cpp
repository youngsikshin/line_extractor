#include <iostream>
#include <ros/ros.h>
#include <line_extractor/LineExtractor.h>

using namespace std;
using namespace line_extractor;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_extractor");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    LineExtractor extractor(nh, private_nh);

    ros::spin();
    
    return 0;
}