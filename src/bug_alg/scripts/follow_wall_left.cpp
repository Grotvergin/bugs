// include ros stuff
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
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
ros::Publisher pub_;
// regions of laser
std::map<std::string, double> regions_ = {{"right", 0}, {"fright", 0}, {"front", 0}, {"fleft", 0}, {"left", 0}};
int state_ = 0;
std::map<int, std::string> state_dict_ = {{0, "find the wall"}, {1, "turn left"}, {2, "follow the wall"}, {3, "turn right"}};

bool wall_follower_left_switch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
void change_state(int state);
void take_action();
geometry_msgs::Twist find_wall();
geometry_msgs::Twist turn_left();
geometry_msgs::Twist turn_right();
geometry_msgs::Twist follow_the_wall();



int main(int argc, char **argv) {
    ros::init(argc, argv, "reading_laser");

    ros::NodeHandle nh;
    pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Subscriber sub = nh.subscribe("/scan", 1000, clbk_laser);

    ros::ServiceServer srv = nh.advertiseService("wall_follower_left_switch", wall_follower_left_switch);

    // main state loop
    ros::Rate rate(20);
    while(ros::ok()) {
        if(!active_) {
            rate.sleep();
            continue;
        }

        geometry_msgs::Twist msg;
        if (state_ == 0) {
            msg = find_wall();
        } else if (state_ == 1) {
            msg = turn_left();
        } else if (state_ == 2) {
            msg = follow_the_wall();
        } else if (state_ == 3) {
            msg = turn_right();
        } else {
            ROS_ERROR("Unknown state!");
        }
        pub_.publish(msg);
        rate.sleep();
    }
}

// switch that can be monitored for results of this service
bool wall_follower_left_switch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    active_ = req.data;
    res.success = true;
    res.message = "Done!";
    return true;
}

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg) {
    regions_["left"] = std::min(*std::min_element(msg->ranges.begin() + 54, msg->ranges.begin() + 89), 10.0f);
    regions_["fleft"] = std::min(*std::min_element(msg->ranges.begin() + 23, msg->ranges.begin() + 53), 10.0f);
    regions_["front"] = std::min(std::min(*std::min_element(msg->ranges.begin(), msg->ranges.begin() + 22), *std::min_element(msg->ranges.begin() + 338, msg->ranges.begin() + 359)), 10.0f);
    regions_["fright"] = std::min(*std::min_element(msg->ranges.begin() + 306, msg->ranges.begin() + 337), 10.0f);
    regions_["right"] = std::min(*std::min_element(msg->ranges.begin() + 270, msg->ranges.begin() + 305), 10.0f);
    regions_["left45"] = std::min(msg->ranges[45], 10.0f);
    take_action();
}

// state changer notifier
void change_state(int state) {
    if (state != state_) {
        std::cout << "Wall follower - [" << state << "] - " << state_dict_[state] << std::endl;
        state_ = state;
    }
}

// main function which tells robot what he should do(chsnges states) depending on his surroundings
void take_action() {
    auto regions = regions_;
    geometry_msgs::Twist msg;
    double linear_x = 0;
    double angular_z = 0;

    std::string state_description = "";

    // not advised to change these settings
    double laser_range = 0.3;
    double diag_range = 0.4;

    if (regions["front"] < laser_range && regions["fleft"] > laser_range && regions["fright"] > laser_range) {
        state_description = "case 1 - front";
        change_state(3);
    } else if (regions["front"] > laser_range && regions["fleft"] > laser_range && regions["fright"] < laser_range) {
        state_description = "case 2 - fright";
        change_state(2);
    } else if (regions["front"] > laser_range && regions["fleft"] > laser_range && regions["fright"] > laser_range && regions["right45"] > diag_range) {
        state_description = "case 3 - correct angle";
        change_state(1);
    } else if (regions["front"] > laser_range && regions["fleft"] < laser_range && regions["fright"] > laser_range) {
        state_description = "case 4 - fleft";
        change_state(2);
    } else if (regions["front"] < laser_range && regions["fleft"] > laser_range && regions["fright"] < laser_range) {
        state_description = "case 5 - front and fright";
        change_state(3);
    } else if (regions["front"] < laser_range && regions["fleft"] < laser_range && regions["fright"] > laser_range) {
        state_description = "case 6 - front and fleft";
        change_state(3);
    } else if (regions["front"] < laser_range && regions["fleft"] < laser_range && regions["fright"] < laser_range) {
        state_description = "case 7 - front and fleft and fright";
        change_state(3);
    } else if (regions["front"] > laser_range && regions["fleft"] < laser_range && regions["fright"] < laser_range) {
       state_description = "case 8 - fleft and fright";
       change_state(0);
    } else {
       state_description = "unknown case";
       ROS_INFO("%s", state_description.c_str());
    }
}

geometry_msgs::Twist find_wall() {
    geometry_msgs::Twist msg;
    msg.linear.x = 0.15;
    return msg;
}

geometry_msgs::Twist turn_left() {
    geometry_msgs::Twist msg;
    msg.angular.z = 0.3;
    return msg;
}

geometry_msgs::Twist turn_right() {
    geometry_msgs::Twist msg;
    msg.angular.z = -0.3;
    return msg;
}

geometry_msgs::Twist follow_the_wall() {
    geometry_msgs::Twist msg;
    msg.linear.x = 0.15;
    return msg;
}