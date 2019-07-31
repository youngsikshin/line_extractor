#include <cassert>
#include <line_extractor/LineSegment.h>

namespace line_extractor {

LineSegment::LineSegment() {
    reset();
}

LineSegment::LineSegment(std_msgs::Header header)
{
    header_ = header;
    reset();
}

LineSegment::LineSegment(line_extractor::LineMessage line_msg)
{
    header_ = line_msg.header;
    rho_ = line_msg.rho;
    theta_ = line_msg.theta;

    num_ = line_msg.num;
    sum_xx_ = line_msg.sum_xx;
    sum_yy_ = line_msg.sum_yy;
    sum_xy_ = line_msg.sum_xy;
    x_bar_ = line_msg.x_bar;
    y_bar_ = line_msg.y_bar;

    sp_ = Point(line_msg.sp.x, line_msg.sp.y);
    ep_ = Point(line_msg.ep.x, line_msg.ep.y);

    raw_sp_ = Point(line_msg.raw_sp.x, line_msg.raw_sp.y);
    raw_ep_ = Point(line_msg.raw_ep.x, line_msg.raw_ep.y);
}

void LineSegment::reset()
{
    rho_ = std::numeric_limits<float>::quiet_NaN();
    theta_ = std::numeric_limits<float>::quiet_NaN();

    num_ = 0;
    sum_xx_ = 0.0f;
    sum_yy_ = 0.0f;
    sum_xy_ = 0.0f;
    x_bar_ = 0.0f;
    y_bar_ = 0.0f;
}

LineMessage LineSegment::to_ros_msg()
{
    LineMessage msg;
    msg.header = header_;
    msg.rho = rho_;
    msg.theta = theta_;

    msg.sp.x = sp_.x();
    msg.sp.y = sp_.y();
    msg.sp.z = 0.0;

    msg.ep.x = ep_.x();
    msg.ep.y = ep_.y();
    msg.ep.z = 0.0;

    msg.raw_sp.x = raw_sp_.x();
    msg.raw_sp.y = raw_sp_.y();
    msg.raw_sp.z = 0.0;

    msg.raw_ep.x = raw_ep_.x();
    msg.raw_ep.y = raw_ep_.y();
    msg.raw_ep.z = 0.0;

    msg.num = num_;
    msg.sum_xx = sum_xx_;
    msg.sum_yy = sum_yy_;
    msg.sum_xy = sum_xy_;

    msg.x_bar = x_bar_;
    msg.y_bar = y_bar_;

    return msg;
}

float LineSegment::calc_angle(LineSegment a, LineSegment b)
{
    float na_x = cosf(a.theta());
    float na_y = sinf(a.theta());

    float nb_x = cosf(b.theta());
    float nb_y = sinf(b.theta());

    float norm_na = std::sqrt(na_x*na_x+na_y*na_y);
    float norm_nb = std::sqrt(nb_x*nb_x+nb_y*nb_y);

    return acosf( (na_x*nb_x+na_y*nb_y)/(norm_na*norm_nb) );
}

void LineSegment::param_update(float x, float y, float theta)
{
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    float cc = cos_theta * cos_theta;
    float ss = sin_theta * sin_theta;
    float cs = cos_theta * sin_theta;

    // tmp variable
    float sum_xx = sum_xx_;
    float sum_xy = sum_xy_;
    float sum_yy = sum_yy_;
    float x_bar = x_bar_;
    float y_bar = y_bar_;

    float num = static_cast<float>(num_);

    sum_xx_ = sum_xx*cc + sum_yy*ss + num*x*x - 2.0*sum_xy*cs + 2.0*num*x*(x_bar*cos_theta - y_bar*sin_theta);
    sum_yy_ = sum_xx*ss + sum_yy*cc + num*y*y + 2.0*sum_xy*cs + 2.0*num*y*(x_bar*sin_theta + y_bar*cos_theta);
    sum_xy_ = cs*(sum_xx-sum_yy) + sum_xy*cosf(2.0*theta) + num*(x_bar*(x*sin_theta+y*cos_theta)+ y_bar*(x*cos_theta-y*sin_theta)+x*y);

    x_bar_ = x_bar*cos_theta - y_bar*sin_theta + x;
    y_bar_ = x_bar*sin_theta + y_bar*cos_theta + y;

    theta_ = 1.0/2.0 * atan2f(-2.0*(sum_xy_-num*x_bar_*y_bar_), (sum_yy_-sum_xx_)-num*(y_bar_*y_bar_ - x_bar_*x_bar_));
    rho_ = x_bar_ * cosf(theta_) + y_bar_ * sin(theta_);

    float px, py;
    px = sp_.x();
    py = sp_.y();

    sp_.x(px*cos_theta-py*sin_theta+x);
    sp_.y(px*sin_theta+py*cos_theta+y);

    px = ep_.x();
    py = ep_.y();

    ep_.x(px*cos_theta-py*sin_theta+x);
    ep_.y(px*sin_theta+py*cos_theta+y);

    px = raw_sp_.x();
    py = raw_sp_.y();

    raw_sp_.x(px*cos_theta-py*sin_theta+x);
    raw_sp_.y(px*sin_theta+py*cos_theta+y);

    px = raw_ep_.x();
    py = raw_ep_.y();

    raw_ep_.x(px*cos_theta-py*sin_theta+x);
    raw_ep_.y(px*sin_theta+py*cos_theta+y);

    float t1 = -raw_sp_.x()*sinf(theta_) + raw_sp_.y()*cosf(theta_);
    float t2 = -raw_ep_.x()*sinf(theta_) + raw_ep_.y()*cosf(theta_);

    Point r1(rho_*cosf(theta_)-t1*sinf(theta_), rho_*sinf(theta_)+t1*cosf(theta_));
    Point r2(rho_*cosf(theta_)-t2*sinf(theta_), rho_*sinf(theta_)+t2*cosf(theta_));

    sp_ = r1;
    ep_ = r2;
}

void LineSegment::merge_line(LineSegment line)
{
    header_ = line.header();
    int src_num = num_;
    int dst_num = line.num();

    num_ = src_num + dst_num;
    sum_xx_ += line.sum_xx();
    sum_yy_ += line.sum_yy();
    sum_xy_ += line.sum_xy();

    x_bar_ = ( static_cast<float>(src_num)*x_bar_ + static_cast<float>(dst_num)*line.x_bar() ) / num_;
    y_bar_ = ( static_cast<float>(src_num)*y_bar_ + static_cast<float>(dst_num)*line.y_bar() ) / num_;

    theta_ = 1.0/2.0 * atan2f(-2.0*(sum_xy_-static_cast<float>(num_)*x_bar_*y_bar_), (sum_yy_-sum_xx_)-static_cast<float>(num_)*(y_bar_*y_bar_ - x_bar_*x_bar_));
    rho_ = x_bar_ * cosf(theta_) + y_bar_ * sin(theta_);

    // point calc (q=map, r=scan)
    Point qr_sp (line.sp().x()-sp_.x(), line.sp().y()-sp_.y());
    Point qr_ep (line.ep().x()-ep_.x(), line.ep().y()-ep_.y());
    Point mj (ep_.x()-sp_.x(), ep_.y()-sp_.y());

    Point new_raw_sp;
    if ( Point::dot_product(qr_sp, mj) >= 0.0 ) {
        new_raw_sp = sp_;
    } else {
        new_raw_sp = line.sp();
    }

    Point new_raw_ep;
    if ( Point::dot_product(qr_ep, mj) <= 0) {
        new_raw_ep = ep_;
    } else {
        new_raw_ep = line.ep();
    }
    
    float t1 = -sinf(theta_)*new_raw_sp.x() + cosf(theta_)*new_raw_sp.y();
    float t2 = -sinf(theta_)*new_raw_ep.x() + cosf(theta_)*new_raw_ep.y();

    Point r1(rho_*cosf(theta_)-t1*sinf(theta_), rho_*sinf(theta_)+t1*cosf(theta_));
    Point r2(rho_*cosf(theta_)-t2*sinf(theta_), rho_*sinf(theta_)+t2*cosf(theta_));

    sp_ = r1;
    ep_ = r2;
}

void LineSegment::print()
{
    std::cout << "rho = " << rho_ << std::endl;
    std::cout << "theta = " << theta_ << std::endl;
    std::cout << "num_ = " << num_ << std::endl;
    std::cout << "sum_xx_ = " << sum_xx_ << std::endl;
    std::cout << "sum_yy_ = " << sum_yy_ << std::endl;
    std::cout << "sum_xy_ = " << sum_xy_ << std::endl;
    std::cout << "x_bar_ = " << x_bar_ << std::endl;
    std::cout << "y_bar_ = " << y_bar_ << std::endl;
}

void LineSegment::update_point(Point a) 
{

    num_++;
    sum_xx_ += a.x()*a.x();
    sum_yy_ += a.y()*a.y();
    sum_xy_ += a.x()*a.y();

    if(num_ == 1) {
        x_bar_ += a.x();
        y_bar_ += a.y();
        raw_sp_ = a;
    } else {
        x_bar_ = (static_cast<float>(num_ - 1)*x_bar_ + a.x())/static_cast<float>(num_);
        y_bar_ = (static_cast<float>(num_ - 1)*y_bar_ + a.y())/static_cast<float>(num_);
        raw_ep_ = a;
    }
    
    if(num_>2) {
        theta_ = 1.0/2.0 * atan2f(-2.0*(sum_xy_-static_cast<float>(num_)*x_bar_*y_bar_), (sum_yy_-sum_xx_)-static_cast<float>(num_)*(y_bar_*y_bar_ - x_bar_*x_bar_));
        rho_ = x_bar_ * cosf(theta_) + y_bar_ * sin(theta_);

        float t1 = -raw_sp_.x()*sinf(theta_) + raw_sp_.y()*cosf(theta_);
        float t2 = -raw_ep_.x()*sinf(theta_) + raw_ep_.y()*cosf(theta_);

        Point r1(rho_*cosf(theta_)-t1*sinf(theta_), rho_*sinf(theta_)+t1*cosf(theta_));
        Point r2(rho_*cosf(theta_)-t2*sinf(theta_), rho_*sinf(theta_)+t2*cosf(theta_));

        sp_ = r1;
        ep_ = r2;

        // std::cout << raw_ep_.x() << ", " << ep_.x() << std::endl;
        // std::cout << raw_ep_.y() << ", " << ep_.y() << std::endl;
        // std::cout << theta_*180.0 / M_PI << std::endl;
        // std::cout << rho_ << std::endl;
    }

}

float LineSegment::dist_from_raw_ep(Point a)
{
    return std::sqrt((raw_ep_.x()-a.x())*(raw_ep_.x()-a.x())+(raw_ep_.y()-a.y())*(raw_ep_.y()-a.y()));
}

float LineSegment::dist_from_ep(Point a)
{
    return std::sqrt((ep_.x()-a.x())*(ep_.x()-a.x())+(ep_.y()-a.y())*(ep_.y()-a.y()));
}

float LineSegment::dist_from_sp(Point a)
{
    return std::sqrt((sp_.x()-a.x())*(sp_.x()-a.x())+(sp_.y()-a.y())*(sp_.y()-a.y()));
}

float LineSegment::dist_from_line(Point a)
{
    // assert(num_ > 2);
    
    // std::cout << num_ << std::endl;
    return fabs(a.x()*cosf(theta_)+a.y()*sinf(theta_)-rho_);
}

}