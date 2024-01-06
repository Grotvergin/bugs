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
#include <fstream>
#include <string>
#include <math.h>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <sys/resource.h>
#include <unordered_map>

// initializing parameters
double yaw_ = 0;
double sum_yaw = 0;
double yaw_error_allowed_ = 5 * (M_PI / 180); // 5 degrees
geometry_msgs::Point position_;
// get the innitial position coordinates
std::unordered_map<std::string, double> regions_;
// states of robot during algorithm
std::array<std::string, 6> state_desc_ = {
    "Go to point",
    "circumnavigate obstacle",
    "go to closest point",
    "turn to circumnavigate left",
    "left_circumnavigating",
    "turn to point/checking for reachability"};
int state_;
double count_state_time_ = 0; // seconds the robot is in a state
double count_loop_ = 0;
double count_point = 0;

ros::ServiceClient srv_client_go_to_point_;
ros::ServiceClient srv_client_wall_follower_;
ros::ServiceClient srv_client_wall_follower_left_;
ros::ServiceClient srv_client_set_model_state;

// 0 - go to point
// 1 - circumnavigate
// 2 - go to closest point
// 3 - turn to circumnavigate left
// 4 - circumnavigate left
// 5 - check reachability



// state changer
void change_state(int state)
{
    int count_state_time = 0;
    state_ = state;
    // informing user that the state has changed
    ROS_INFO("%s", "state changed: " + state_desc_[state]);
    // different states turn on and off different servers(other scripts)
    switch (state_)
    {
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

// callbacks
// robot movement callbacks
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

// laser callback
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg) {
    regions_["left"] = std::min(*std::min_element(msg->ranges.begin() + 54, msg->ranges.begin() + 89), 10.0f);
    regions_["fleft"] = std::min(*std::min_element(msg->ranges.begin() + 23, msg->ranges.begin() + 53), 10.0f);
    regions_["front"] = std::min(std::min(*std::min_element(msg->ranges.begin(), msg->ranges.begin() + 22), *std::min_element(msg->ranges.begin() + 338, msg->ranges.begin() + 359)), 10.0f);
    regions_["fright"] = std::min(*std::min_element(msg->ranges.begin() + 306, msg->ranges.begin() + 337), 10.0f);
    regions_["right"] = std::min(*std::min_element(msg->ranges.begin() + 270, msg->ranges.begin() + 305), 10.0f);
    regions_["left45"] = std::min(msg->ranges[45], 10.0f);
    take_action();
}


// function to calculate distance betweeen two points
double calc_dist_points(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    double dist_y = point1.y - point2.y;
    double dist_x = point1.x - point2.x;
    return sqrt(pow(dist_y, 2) + pow(dist_x, 2));
}

// function to normalize angle
double normalize_angle(double angle)
{
    if (fabs(angle) > M_PI)
        angle = angle - (2 * M_PI * angle) / (fabs(angle));
    return angle;
}

std::string vector_to_string(std::vector<geometry_msgs::Point> vect_pnts)
{
    std::string str;
    str.append("[");
    for (auto i : vect_pnts)
    {
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
    geometry_msgs::Point circumnavigate_starting_point_;
    geometry_msgs::Point circumnavigate_closest_point_;
    geometry_msgs::Point prev_point_;
    geometry_msgs::Point initial_position_;
    ros::param::get("initial_x", initial_position_.x);
    ros::param::get("initial_y", initial_position_.y);
    initial_position_.z = 0;
    // get the destination coordinates
    geometry_msgs::Point desired_position_;
    ros::param::get("des_pos_x", desired_position_.x);
    ros::param::get("des_pos_y", desired_position_.y);
    desired_position_.z = 0;
    double sum_yaw = 0;
    std::vector<geometry_msgs::Point> points, POINTS;
    std::vector<double> length_to_points;
    auto timer_hp = std::chrono::system_clock::now();
    int obstacle_count = 0;

    // init script as a node
    ros::init(argc, argv, "bug1");
    // creating publisher to change velocities
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/cmd_vel", 1);
    ros::ServiceClient reset_world = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
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

    // set robot position configure it as burger robot
    gazebo_msgs::ModelState model_state;
    model_state.model_name = "turtlebot3_burger";

    // calculate the yaw to destination point
    double desired_yaw;
    desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
    double err_yaw = desired_yaw - yaw_;

    double pth = 0;

    // point the robot to the destination point

    while (!(fabs(err_yaw) <= M_PI / 90))
    {
        geometry_msgs::Twist twist_msg;
        desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
        err_yaw = desired_yaw - yaw_;
        if (err_yaw > 0)
            twist_msg.angular.z = 0.7;
        else
            twist_msg.angular.z = -0.7;
        pub.publish(twist_msg);
        twist_msg.angular.z = 0;
        pub.publish(twist_msg);
    }

    // initialize going to the point
    change_state(0);
    double yaw_before = yaw_;

    int rate_hz = 20;
    ros::Rate rate(rate_hz);

    // circle to change robot states

    while (!(ros::isShuttingDown()))
    {
        int num_points;
        if (regions_.empty())
            continue;
        double diff = fabs(yaw_ - yaw_before);

        if (diff > 1)
            sum_yaw += fabs(yaw_ + yaw_before);
        else
            sum_yaw += fabs(yaw_before - yaw_);
        yaw_before = yaw_;

        // check if robot arrived at the dest point

        if (sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2)) < 0.2)
        {
            ROS_INFO("%s", "point reached");
            geometry_msgs::Twist twist_msg;
            twist_msg.angular.z = 0;
            twist_msg.linear.x = 0;
            pub.publish(twist_msg);
            POINTS.push_back(position_);
            ROS_INFO("%s", "point reached");
            auto RESULT_TIME = std::chrono::system_clock::now() - timer_hp;

            std::ofstream file("results.txt");
            if (!file.is_open())
            {
                exit(1);
            }
            file << "ELAPSED TIME " << RESULT_TIME.count() << " ns\n";
            file << "OBSTACLES COUNT: " + std::to_string(obstacle_count) << "\n";
            file << "POINTS: " << vector_to_string(POINTS) << "\n";
            struct rusage usage;
            getrusage(RUSAGE_SELF, &usage);
            file << "MEMORY USAGE: " << std::to_string(usage.ru_maxrss) << "Kb\n";
            file << "COMPLEXITY: EASY"
                 << "\n";
            file << "CALCULATION TIME: -"
                 << "\n";
            file << "TOTAL TURN: " + std::to_string(sum_yaw) + "\n";
            file.close();
            system("rosnode kill /go_to_point");
            system("rosnode kill /wall_follower");
            system("rosnode kill /wall_follower_left");
            std_srvs::Empty srv;
            reset_world.call(srv);
            std::string str = "kill " + std::to_string(getpid());
            system(str.c_str());
        }
        // go to point state
        switch (state_)
        {
        case 0:{
            bool right = true;
            desired_yaw = atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x);
            err_yaw = desired_yaw - yaw_;
            // check if there is an obstacle forward
            if (regions_["front"] > 0.15 && regions_["front"] < 0.3)
            {
                circumnavigate_closest_point_ = position_;
                circumnavigate_starting_point_ = position_;
                // delete the array of points described as leave points if robot has visited an obstacle before
                points.push_back(position_);
                length_to_points.push_back(0);
                prev_point_ = position_;
                // start circumnavigating
                obstacle_count += 1;
                change_state(1);
            }
            break;
        }
        case 1:
        {
            // compare only after 5 seconds - need some time to get out of starting_point
            // if robot reaches (is close to) starting point

            if (count_state_time_ > 20 && calc_dist_points(position_, circumnavigate_starting_point_) < 0.2)
            {
                bool right = true;
                double least_path = pth;
                // choosing the leave point and the direction
                for (int i = 0; i < num_points; i++)
                {
                    if (length_to_points[i] < least_path)
                    {
                        circumnavigate_closest_point_ = points[i];
                        least_path = length_to_points[i];
                        right = true;
                    }
                    if (pth - length_to_points[i] < least_path)
                    {
                        circumnavigate_closest_point_ = points[i];
                        least_path = pth - length_to_points[i];
                        right = false;
                    }
                }
                // if closest point is back of the robot regarding the direction of circumnavigating start turning back
                if (!right)
                {
                    geometry_msgs::Twist twist_msg;
                    twist_msg.linear.x = 0;
                    twist_msg.angular.z = 0;
                    pth = 0;
                    points.clear();
                    length_to_points.clear();
                    num_points = 0;
                    change_state(3);
                }
                else
                {
                    geometry_msgs::Twist twist_msg;
                    twist_msg.linear.x = 0;
                    pth = 0;
                    points.clear();
                    length_to_points.clear();
                    num_points = 0;
                    change_state(2);
                }
            }

            // calculate path to point (perimeter)
            pth += calc_dist_points(position_, prev_point_);
            prev_point_ = position_;
            // if current position is closer to the goal than the previous closest_position, assign current position to closest_point

            if (calc_dist_points(position_, desired_position_) < calc_dist_points(circumnavigate_closest_point_, desired_position_))
            {
                points.clear();
                length_to_points.clear();
                num_points = 1;
                points.push_back(position_);
                length_to_points.push_back(pth);
                circumnavigate_closest_point_ = position_;
            }
            else if (calc_dist_points(position_, desired_position_) < calc_dist_points(circumnavigate_closest_point_, desired_position_))
            {
                num_points += 1;
                points.push_back(position_);
                length_to_points.push_back(pth);
            }
            break;
        }
        case 2:
        {
            // if robot reaches (is close to) closest point

            if (calc_dist_points(position_, circumnavigate_closest_point_) < 0.2)
                change_state(5);

            break;
        }
        // face left
        case 3:
        {
            while (regions_["left45"] > 0.4)
            {
                geometry_msgs::Twist twist_msg;
                twist_msg.angular.z = 0.3;
                pub.publish(twist_msg);
            }
            change_state(4);
            break;
        }
        // circumnavigate left until the closest point is near
        case 4:
        {
            if (calc_dist_points(position_, circumnavigate_closest_point_) < 0.2)
                change_state(5);
            break;

            // check if path from the closest point is clear
        }
        case 5:
        {
            desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
            err_yaw = desired_yaw - yaw_;
            while (!(fabs(err_yaw) <= M_PI / 90))
            {
                geometry_msgs::Twist twist_msg;
                desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
                err_yaw = desired_yaw - yaw_;
                if (err_yaw > 0)
                    twist_msg.angular.z = 0.7;
                else
                    twist_msg.angular.z = -0.7;
                pub.publish(twist_msg);
                twist_msg.angular.z = 0;
                pub.publish(twist_msg);
                diff = fabs(yaw_ - yaw_before);
                if (diff > 1)
                    sum_yaw += fabs(yaw_ + yaw_before);
                else
                    sum_yaw += fabs(yaw_before - yaw_);
                yaw_before = yaw_;
            }
            if (regions_["front"] > 0.15 && regions_["front"] < 0.45)
            {
                POINTS.push_back(position_);
                ROS_INFO("%s", "point cannot be reached");
                auto RESULT_TIME = std::chrono::system_clock::now() - timer_hp;

                std::ofstream file("results.txt");
                if (!file.is_open())
                {
                    exit(1);
                }
                file << "ELAPSED TIME " << RESULT_TIME.count() << " ns\n";
                file << "OBSTACLES COUNT: " + std::to_string(obstacle_count) << "\n";
                file << "POINTS: " << vector_to_string(POINTS) << "\n";
                struct rusage usage;
                getrusage(RUSAGE_SELF, &usage);
                file << "MEMORY USAGE: " << std::to_string(usage.ru_maxrss) << "Kb\n";
                file << "COMPLEXITY: EASY"
                     << "\n";
                file << "CALCULATION TIME: -"
                     << "\n";
                file << "TOTAL TURN: " + std::to_string(sum_yaw) + "\n";
                file.close();
                system("rosnode kill /go_to_point");
                system("rosnode kill /wall_follower");
                system("rosnode kill /wall_follower_left");
                std_srvs::Empty srv;
                reset_world.call(srv);
                std::string str = "kill " + std::to_string(getpid());
                system(str.c_str());
            }
            change_state(0);
            break;
        }
        }

        count_loop_ = count_loop_ + 1;

        if (count_loop_ == 20)
        {
            count_state_time_ = count_state_time_ + 1;
            count_loop_ = 0;
        }
        count_loop_ = count_loop_ + 1;
        if (count_point == 25)
        {
            count_point = 0;
            POINTS.push_back(position_);
        }
        rate.sleep();
    }
}