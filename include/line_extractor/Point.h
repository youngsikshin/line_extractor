#pragma once

#include <iostream>
#include <cmath>

namespace line_extractor
{

class Point
{
public:    
    Point() { x_ = 0.0; y_ = 0.0; }
    Point(const Point& a) { x_ = a.x(); y_= a.y(); }
    Point(float x, float y)
        : x_(x), y_(y) 
    {

    }
    ~Point() {}

    static float calc_distance(Point a, Point b);
    static float calc_angle(Point a, Point b);
    static float dot_product(Point a, Point b);
    static float length(Point a, Point b);

    float calc_distance_from(Point a);
    float norm();

    float x() const { return x_; }
    float y() const { return y_; }
    void x(float x) { x_ = x; }
    void y(float y) { y_ = y; }

    void print() const { std::cout << "Point( " << x_ << ", " << y_ << ")" << std::endl; }

    Point& operator=(const Point& rhs);

private:
    float x_;
    float y_;
};

}
