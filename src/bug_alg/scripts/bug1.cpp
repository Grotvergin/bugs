#include "ros/ros.h"
#include "ros/console.h"
#include "ros/param.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_msgs/String.h"
#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/SetBoolRequest.h>
#include <std_srvs/SetBoolResponse.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>
#include <iostream>
#include <string>
#include <math.h>
#include <chrono>
#include <ctime>
#include <algorithm>

// initializing parameters
bool srv_client_go_to_point_;
bool srv_client_wall_follower_;
bool srv_client_wall_follower_left_;
float yaw_;
float sum_yaw;
float yaw_error_allowed_ = 5 * (M_PI / 180); // 5 degrees
// get the innitial position coordinates
bool regions_;
// states of robot during algorithm
std::array<std::string, 6> state_desc_ = {
    "Go to point",
    "circumnavigate obstacle",
    "go to closest point",
    "turn to circumnavigate left",
    "left_circumnavigating",
    "turn to point/checking for reachability"};
int state_;
float count_state_time_ = 0; // seconds the robot is in a state
float count_loop_ = 0;
float count_point = 0;

ros::ServiceClient srv_client_go_to_point_;
ros::ServiceClient srv_client_wall_follower_;
ros::ServiceClient srv_client_wall_follower_left_;
ros::ServiceClient srv_client_set_model_state;
ros::ServiceClient reset_world;

// 0 - go to point
// 1 - circumnavigate
// 2 - go to closest point
// 3 - turn to circumnavigate left
// 4 - circumnavigate left
// 5 - check reachability

// callbacks
// robot movement callbacks
void clbk_odom(const std_msgs::StringConstPtr& msg){
    // foo
}
// laser callback
void clbk_laser(const std_msgs::StringConstPtr& msg){
    // foo
}

// state changer
void change_state(int state){
    int count_state_time = 0;
    state_ = state;
    // informing user that the state has changed
    ROS_INFO("state changed: ", state_desc_[state]);
    // different states turn on and off different servers(other scripts)
    switch(state_){
        case 0:
            srv_client_go_to_point_ = true;
            srv_client_wall_follower_ = false;
            srv_client_wall_follower_left_ = false;
            break;
        case 1:
            srv_client_go_to_point_ = false;
            srv_client_wall_follower_ = true;
            srv_client_wall_follower_left_ = false;
            break;
        case 2:
            srv_client_go_to_point_ = false;
            srv_client_wall_follower_ = true;
            srv_client_wall_follower_left_ = false;
            break;
        case 3:
            srv_client_go_to_point_ = false;
            srv_client_wall_follower_ = false;
            srv_client_wall_follower_left_ = false;
            break;
        case 4:
            srv_client_go_to_point_ = false;
            srv_client_wall_follower_ = false;
            srv_client_wall_follower_left_ = true;
            break;
        case 5:
            srv_client_go_to_point_ = false;
            srv_client_wall_follower_ = false;
            srv_client_wall_follower_left_ = false;
            break;
    }
}

// function to calculate distance betweeen two points
float calc_dist_points(geometry_msgs::Point point1, geometry_msgs::Point point2){
    float dist_y = point1.y - point2.y;
    float dist_x = point1.x - point2.x;
    return sqrt(pow(dist_y, 2) + pow(dist_x, 2));
}

// function to normalize angle
float normalize_angle(float angle){
    if (fabs(angle) > M_PI) angle = angle - (2 * M_PI * angle) / (fabs(angle));
    return angle;
}

std::string vector_to_string(std::vector<geometry_msgs::Point> vect_pnts){
    std::string str;
    str.append("[");
    for (auto i : vect_pnts){
        str.append(std::to_string(i.x));
        str.append(" ");
        str.append(std::to_string(i.y));
        str.append(";");
    }
    str.append("]");
    return str;
}

int main(int argc, char **argv)
{
    geometry_msgs::Point circumnavigate_starting_point_();
    geometry_msgs::Point circumnavigate_closest_point_();
    geometry_msgs::Point position_;
    geometry_msgs::Point initial_position_;
    ros::param::get("initial_x", initial_position_.x);
    ros::param::get("initial_y", initial_position_.y);
    initial_position_.z = 0;
    // get the destination coordinates
    geometry_msgs::Point desired_position_;
    ros::param::get("des_pos_x", desired_position_.x);
    ros::param::get("des_pos_y", desired_position_.y);
    desired_position_.z = 0;
    float sum_yaw = 0;
    std::vector<geometry_msgs::Point> points, POINTS;
    std::vector<float> length_to_points;
    auto timer_hp = std::chrono::system_clock::now();
    int obstacle_count = 0;

    // init script as a node
    ros::init(argc, argv, "bug1");
    // creating publisher to change velocities
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/cmd_vel", 1);

    ros::Subscriber sub_laser = nh.subscribe("/scan", 1, clbk_laser);
    ros::Subscriber sub_odom = nh.subscribe("/scan", 1, clbk_odom); // вроде не используются, передаются функции заглушки

    // initialize servers for other scripts involved
    ros::service::waitForService("/go_to_point_switch");
    ros::service::waitForService("/wall_follower_switch");
    ros::service::waitForService("/wall_follower_left_switch");
    ros::service::waitForService("/gazebo/set_model_state");

    srv_client_go_to_point_ = nh.serviceClient<std_srvs::SetBool>("/go_to_point_switch");
    srv_client_wall_follower_ = nh.serviceClient<std_srvs::SetBool>("/wall_follower_switch");
    srv_client_wall_follower_left_ = nh.serviceClient<std_srvs::SetBool>("/wall_follower_left_switch");
    srv_client_set_model_state = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    reset_world = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");


}