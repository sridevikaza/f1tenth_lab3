#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include "rclcpp/parameter.hpp"
#include <optional>

using std::placeholders::_1;
using namespace std;


class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // declare parameters
        // this->declare_parameter<double>("kp", 0.0);

        // scan sub
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, _1));

       // drive pub
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

    }

private:
    // PID gains
    double kp = 1.1;
    double kd = 0.8;
    double ki = 0.001;

    // params
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double theta = M_PI/4;
    double desired_dist = 1.0;
    double look_ahead_dist = 0.2;
    rclcpp::Time t = this->now();
    rclcpp::Time prev_t = this->now();

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;


    double get_range(const float* range_data, double angle, double angle_min, double angle_increment)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR
            angle_min: min angle of the LiDAR scan
            angle_increment: angular increment of the LiDAR scan

        Returns:
            range: range measurement in meters at the given angle
        */

        // get index corresponding to the provided angle
        double angle_diff = angle - angle_min;
        int index = static_cast<int>(angle_diff / angle_increment);
        float range = range_data[index];

        // check nans and infs
        if (!isnan(range) && !isinf(range))
        {
            return static_cast<double>(range);
        }

        // return; // return 0 if the range is inf/nan/oob
        return numeric_limits<double>::quiet_NaN();
    }


    double get_error(const float* range_data, double angle_min, double angle_increment)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            desired_dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // get two laser scans
        double a = get_range(range_data, M_PI/2-theta, angle_min, angle_increment);
        double b = get_range(range_data, M_PI/2, angle_min, angle_increment);
        t = this->now();

        if (!isnan(a) && !isnan(b)) {

            double alpha = atan2((a*cos(theta) - b), (a*sin(theta)));
            double dist = b*cos(alpha);
            double future_dist = dist + look_ahead_dist*sin(alpha);
            double error = desired_dist - future_dist;

            return -error;
        }

        return std::numeric_limits<double>::quiet_NaN();
    }

    void pid_control(double error)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */

        // use kp, ki & kd to implement a PID controller
        double proportional = error;
        integral += error;
        double derivative = (error - prev_error) / (10e9*(t-prev_t).nanoseconds());
        double angle = kp*proportional + ki*integral + kd*derivative;
        prev_error = error;
        prev_t = t;

        // calculate velocity
        double velocity = 0.0;
        if (angle < 10*M_PI/180){
            velocity = 1.5;
        }
        else if (angle < 20*M_PI/180){
            velocity = 1.0;
        }
        else{
            velocity = 0.5;
        }

        // fill in drive message and publish
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = angle;
        drive_publisher_->publish(drive_msg);
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */

       // get error
        double error = get_error(scan_msg->ranges.data(), scan_msg->angle_min, scan_msg->angle_increment);

        // actuate the car with PID
        if (!isnan(error)) {
            pid_control(error);
        }
    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}