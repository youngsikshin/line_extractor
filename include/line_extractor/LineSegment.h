#include <iostream>
#include <cmath>
#include <limits>

#include <line_extractor/Point.h>
#pragma once
#include <std_msgs/Header.h>
#include <line_extractor/LineMessage.h>

namespace line_extractor
{

class LineSegment
{
public:
    LineSegment();
    LineSegment(std_msgs::Header header);
    LineSegment(line_extractor::LineMessage line_msg);
    ~LineSegment() {}
    void reset();
    void print();
    float length() { return sqrt((sp_.x()-ep_.x())*(sp_.x()-ep_.x()) + (sp_.y()-ep_.y())*(sp_.y()-ep_.y())); }
    static float calc_angle(LineSegment a, LineSegment b);
    void update_point(Point a);
    void merge_line(LineSegment line);
    float dist_from_raw_ep(Point a);
    float dist_from_ep(Point a);
    float dist_from_sp(Point a);
    float dist_from_line(Point a);
    Point& sp() {return sp_;}
    Point& ep() {return ep_;}
    float theta() {return theta_;}
    float rho() { return rho_; }
    int num() const {return num_;}
    float sum_xx() const {return sum_xx_;}
    float sum_yy() const {return sum_yy_;}
    float sum_xy() const {return sum_xy_;}
    float x_bar() const {return x_bar_;}
    float y_bar() const {return y_bar_;}
    std_msgs::Header& header() { return header_; }
    LineMessage to_ros_msg();
    void param_update(float x, float y, float theta);
    double get_time_diff(std_msgs::Header header) {
        return abs(header_.stamp.toSec() - header.stamp.toSec());
    }

private:
    std_msgs::Header header_;
    float rho_;
    float theta_;
    Point sp_;
    Point ep_;
    Point raw_sp_;
    Point raw_ep_;

    int num_;
    float sum_xx_;
    float sum_yy_;
    float sum_xy_;
    float x_bar_;
    float y_bar_;
};

}