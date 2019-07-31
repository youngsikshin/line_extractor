#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <line_extractor/LineSegment.h>

using namespace std;

namespace line_extractor
{

class LineExtractor
{
public:
    LineExtractor(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~LineExtractor() { }
    void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
    void reset();

private:
    ros::Publisher pub_;
    ros::Publisher pc_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber pc_sub_;
    
    bool is_need_seed_;
    vector<LineSegment> line_segments_;
};

}