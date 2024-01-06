// import ros stuff
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/SetBoolRequest.h"
#include "std_srvs/SetBoolResponse.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/TriggerRequest.h"
#include "std_srvs/TriggerResponse.h"

#include <cmath>

bool active_ = false;

// robot state variables
geometry_msgs::Point position_;
double yaw_ = 0;
// machine state
int state_ = 0;
// goal is declared in main

// parameters
float yaw_precision_ = 2 * M_PI / 90;
float dist_precision_ = 0.1;

// publishers
ros::Publisher pub;

bool go_to_point_switch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
void clbk_odom(const nav_msgs::Odometry::ConstPtr& msg);
void change_state(int state);
double normalize_angle(double angle);
void fix_yaw(geometry_msgs::Point des_pos);
void go_straight_ahead(geometry_msgs::Point des_pos);
void done();


int main(int argc, char **argv) {
    geometry_msgs::Point desired_position;
    ros::param::get("des_pos_x", desired_position.x);
    ros::param::get("des_pos_y", desired_position.y);
    desired_position.z = 0;

    // initializing node, publishers and services
    ros::init(argc, argv, "go_to_point");

    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Subscriber sub_odom = nh.subscribe("/odom", 1, clbk_odom);

    ros::ServiceServer srv = nh.advertiseService("go_to_point_switch", go_to_point_switch);

    ros::Rate rate(40);

    double desired_yaw = atan2(desired_position.y - position_.y, desired_position.x - position_.x);
    double err_yaw = desired_yaw - yaw_;

    while (fabs(err_yaw) > M_PI / 90) {
        geometry_msgs::Twist twist_msg;
        desired_yaw = atan2(desired_position.y - position_.y, desired_position.x - position_.x);
        err_yaw = desired_yaw - yaw_;
        twist_msg.angular.z = (err_yaw > 0 ? 0.7 : -0.7);
        pub.publish(twist_msg);
        twist_msg.angular.z = 0;
        pub.publish(twist_msg);
    }

    // main loop for robot states
    while(ros::ok()) {
        if (!active_) 
            continue;
        else {
            if (state_ == 0)
                fix_yaw(desired_position);
            else if (state_ == 1)
                go_straight_ahead(desired_position);
            else if (state_ == 2)
                done();
            else
                ROS_ERROR("Unknown state!");
        }

        ros::spinOnce();
        rate.sleep();
    }
        
    return EXIT_SUCCESS;
}

// service callbacks
bool go_to_point_switch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    active_ = req.data;
    res.success = true;
    res.message = "Done!";
    return true;
}

// movement callbacks
void clbk_odom(const nav_msgs::Odometry::ConstPtr& msg) {
    // position
    position_ = msg->pose.pose.position;

    // yaw
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);
}

// state changer outputs state of the robot
void change_state(int state) {
    state_ = state;
    std::cout << "State changed to [" << state_ << "]" << std::endl;
}

double normalize_angle(double angle) {
    if (std::fabs(angle) > M_PI) 
        angle = angle - (2 * M_PI * angle) / (std::fabs(angle));
    return angle;
}

// turning in the right direction if for some reason the robot has been turned away
void fix_yaw(geometry_msgs::Point des_pos) {
    double desired_yaw = atan2(des_pos.y - position_.y, des_pos.x - position_.x);
    double err_yaw = normalize_angle(desired_yaw - yaw_);

    ROS_INFO("%f", err_yaw);

    geometry_msgs::Twist twist_msg;

    // fixing yaw
    if (fabs(err_yaw) > yaw_precision_)
        twist_msg.angular.z = err_yaw > 0 ? 0.2 : -0.2;
    
    pub.publish(twist_msg);

    if (fabs(err_yaw) <= yaw_precision_) {
        ROS_INFO("Yaw error: [%f]", err_yaw);
        twist_msg.angular.z = 0;
        pub.publish(twist_msg);
        change_state(1);
    }
}

// function that checks constanly if the robot is on the right way and whether he reached the point to stop there
void go_straight_ahead(geometry_msgs::Point des_pos) {
    double desired_yaw = atan2(des_pos.y - position_.y, des_pos.x - position_.x);
    double err_yaw = normalize_angle(desired_yaw - yaw_);
    double err_pos = sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2));

    if (err_pos > dist_precision_) {
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = 0.2;
        pub.publish(twist_msg);
    } else {
        ROS_INFO("Position error: [%f]", err_pos);
        // stop at finish
        change_state(2);
    }

    // State change conditions/ if robot is looking the wrong way we go to turning him back
    if (fabs(err_yaw) > yaw_precision_) {
        ROS_INFO("Yaw error: [%f]", err_yaw);
        change_state(0);
    }
}

void done() {
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0;
    twist_msg.angular.z = 0;
    pub.publish(twist_msg);
}

