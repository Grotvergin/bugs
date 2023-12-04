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

// 0 - go to point
// 1 - circumnavigate
// 2 - go to closest point
// 3 - turn to circumnavigate left
// 4 - circumnavigate left
// 5 - check reachability


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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bug1");
    ros::NodeHandle nodeHandler;
    ros::Publisher pub = nodeHandler.advertise<std_msgs::String>("/cmd_vel", 5); // не знаю почему 5
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
}