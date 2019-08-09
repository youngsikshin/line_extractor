#include <line_extractor/LineExtractor.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/Marker.h>
#include <line_extractor/LineListMessage.h>

namespace line_extractor {

LineExtractor::LineExtractor(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    private_nh.param<std::string>("sub_pc_topic", sub_pc_topic_, "/filtered_pc");
    private_nh.param<std::string>("pub_line_topic", pub_line_topic_, "/lines");
    private_nh.param<std::string>("filtered_pc_topic", pub_line_filtered_pc_topic_, "line_pc");
    private_nh.param<std::string>("line_visualize_topic", line_vis_topic_, "/visualization_marker");

    private_nh.param<double>("line_distance", line_distance_threshold_, 0.12);
    private_nh.param<double>("angular_resolution", angular_resolution_, 6.8);
    private_nh.param<double>("arc_length_ratio", arc_length_ratio_, 1.1);

    pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(sub_pc_topic_,1,&LineExtractor::scan_callback, this);
    pub_ =  nh.advertise<line_extractor::LineListMessage>(pub_line_topic_, 10);
    pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_line_filtered_pc_topic_, 10);
    
    marker_pub_ = nh.advertise<visualization_msgs::Marker>(line_vis_topic_, 10);
    reset();
}

void LineExtractor::reset()
{
    is_need_seed_ = true;
    line_segments_.clear();
}

void LineExtractor::scan_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_msg, *pc_ptr);

    if(pc_ptr->points.size() < 3)   return;

    pcl::PointCloud <pcl::PointXYZ> pc_cloud;
    pcl::PointCloud <pcl::PointXYZ> tmp_cloud;

    reset();

    float angle_thresh = angular_resolution_ * M_PI/180.0;
    float line_dist_threshold = line_distance_threshold_;

    size_t idx = 0;
    do {
        if(is_need_seed_) {
            if (!(idx < pc_ptr->points.size()-2)) break;

            LineSegment seed_seg(pc_msg->header);

            Point point_k(pc_ptr->points[idx].x, pc_ptr->points[idx].y);
            Point point_k2(pc_ptr->points[idx+1].x, pc_ptr->points[idx+1].y);
            Point point_k3(pc_ptr->points[idx+2].x, pc_ptr->points[idx+2].y);
            
            float dist1 = Point::calc_distance(point_k, point_k2);
            float dist2 = Point::calc_distance(point_k2, point_k3);

            float angle1 = Point::calc_angle(point_k, point_k2);
            float angle2 = Point::calc_angle(point_k2, point_k3);

            float threshold = point_k2.norm()*angle_thresh*arc_length_ratio_;
            // float threshold = 0.0;
            // if(point_k2.norm() < 1.5)
            //     threshold = point_k2.norm()*angle_thresh*1.5;
            // else
            //     threshold = point_k2.norm()*angle_thresh*1.8;

            // std::cout << angle1*180.0/M_PI << ", " << angle2*180.0/M_PI << std::endl;
            if(std::isnan(angle1)) {
                std::cout << "angle1 is nan" << std::endl;
                point_k.print();
                point_k2.print();
            }

            if(std::isnan(angle2)) {
                std::cout << "angle2 is nan" << std::endl;
                point_k2.print();
                point_k3.print();
            }
            // std::cout << angle1 << ", " << angle2 << std::endl;
            // std::cout << dist1 << ", " << dist2 << ", " << threshold << std::endl;

            if(dist1 < threshold && dist2 < threshold) {
                seed_seg.update_point(point_k);
                seed_seg.update_point(point_k2);
                seed_seg.update_point(point_k3);

                is_need_seed_ = false;
                line_segments_.push_back(seed_seg);

                idx = idx+3;
                
                std::cout << "initialized seed line" << std::endl;

                tmp_cloud.points.clear();
                tmp_cloud.points.push_back(pc_ptr->points[idx]);
                tmp_cloud.points.push_back(pc_ptr->points[idx+1]);
                tmp_cloud.points.push_back(pc_ptr->points[idx+2]);
            } else {
                // std::cout << "failed seed" << std::endl;
                idx = idx+1;
            }
        }
        else {
            LineSegment& last_line_seg = line_segments_[line_segments_.size()-1];

            Point point_k(pc_ptr->points[idx].x, pc_ptr->points[idx].y);
            float point_dist = last_line_seg.dist_from_raw_ep(point_k);
            float line_dist = last_line_seg.dist_from_line(point_k);

            float point_dist_threshold = point_k.norm()*angle_thresh*arc_length_ratio_;
            // float point_dist_threshold = 0.0f;
            // if(point_k.norm() < 1.5)
            //     point_dist_threshold = point_k.norm()*angle_thresh*1.5;
            // else
            //     point_dist_threshold = point_k.norm()*angle_thresh*1.8;

            std::cout << "point threshold: " << point_dist_threshold << ", " << "line distance: " << line_dist_threshold << std::endl;
            std::cout << "point distance : " << point_dist << ", " << "line distance: " << line_dist << std::endl;

            if(point_dist < point_dist_threshold && line_dist < line_dist_threshold) {
                std::cout << "update last line segment" << std::endl;
                last_line_seg.update_point(point_k);
                idx = idx+1;

                if(tmp_cloud.size() == 3) {
                    pc_cloud += tmp_cloud;
                    tmp_cloud.clear();
                }
                    
                pc_cloud.points.push_back(pc_ptr->points[idx]);
            }else{
                is_need_seed_ = true;
            }
        }

        
    } while(idx < pc_ptr->points.size());

    is_need_seed_ = true;

    // publish linelist
    LineListMessage lines;
    lines.header = pc_msg->header;

    for(LineSegment line_segment:line_segments_) {
        lines.lines.push_back(line_segment.to_ros_msg());
    }
    pub_.publish(lines);

    // publish filtered pc
    pc_cloud.height = 1;
    pc_cloud.width = pc_cloud.points.size();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(pc_cloud, cloud_msg);
    cloud_msg.header = pc_msg->header;
    pc_pub_.publish(cloud_msg);
    
    // Publish visualization msg
    visualization_msgs::Marker line_list;
    line_list.header = pc_msg->header;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;

    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    for(LineSegment line_segment:line_segments_)
    {
        if(line_segment.num() < 4) continue;

        geometry_msgs::Point sp_msg, ep_msg;
        sp_msg.x = line_segment.sp().x();
        sp_msg.y = line_segment.sp().y();
        ep_msg.x = line_segment.ep().x();
        ep_msg.y = line_segment.ep().y();

        line_list.points.push_back(sp_msg);
        line_list.points.push_back(ep_msg);
    }

    marker_pub_.publish(line_list);
}

}