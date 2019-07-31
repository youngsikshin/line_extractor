#include <line_extractor/Point.h>

namespace line_extractor
{

float Point::calc_distance(Point a, Point b)
{
    return std::sqrt( (a.x_-b.x_)*(a.x_-b.x_) + (a.y_-b.y_)*(a.y_-b.y_));
}

float Point::calc_angle(Point a, Point b)
{
    return acosf(dot_product(a, b)/(a.norm()*b.norm()));
}

float Point::dot_product(Point a, Point b)
{
    return a.x_*b.x_+a.y_*b.y_;
}

float Point::length(Point a, Point b)
{
    return sqrt((a.x_-b.x_)*(a.x_-b.x_)-(a.y_-b.y_)*(a.y_-b.y_));
}

float Point::calc_distance_from(Point a)
{
    return sqrt((a.x_-x_)*(a.x_-x_)-(a.y_-y_)*(a.y_-y_));
}

float Point::norm()
{
    return std::sqrt(x_*x_+y_*y_);
}

Point& Point::operator=(const Point& rhs)
{
    x_ = rhs.x();
    y_ = rhs.y();

    
    return *this;
}

};
