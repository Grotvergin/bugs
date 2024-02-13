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

// Definitions for some initial parameters
#define YAW_PRECISION 2 * M_PI / 90
#define DIST_PRECISION 0.1
#define LASER_RANGE 0.3
#define DIAG_RANGE 0.4

#ifndef DIR_IS_LEFT
#define DIR_IS_LEFT true
#endif

#ifndef NORMAL_SPEED_FORWARD
#define NORMAL_SPEED_FORWARD 0.2
#endif

#ifndef SLOW_SPEED_FORWARD
#define SLOW_SPEED_FORWARD 0.1
#endif

#ifndef ROTATE_SPEED
#define ROTATE_SPEED 0.15
#endif

#ifndef ACCURACY_TARGET
#define ACCURACY_TARGET 0.2
#endif

#ifndef RESULT_FILE_NAME
#define RESULT_FILE_NAME "results.txt"
#endif

#ifndef ALG_COMPLEXITY
#define ALG_COMPLEXITY "medium"
#endif

class BugAlg {
public:
    BugAlg();
    virtual ~BugAlg() {};
    virtual void main_logic() {};

protected:
    // Variables for ROS subscribing/publishing
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::ServiceClient reset_world, srv_client_set_model_state;
    ros::Subscriber sub_laser, sub_odom;

    // Variable for tracking algorithm execution time
    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

    // Variable that stores the minimum distances between the robot
    // and the obstacle in several key directions
    std::unordered_map<std::string, double> regions;

    // Variables for calculation of the results
    int obstacle_count = 0;
    double overall_distance = 0, prev_yaw = 0, sum_yaw = 0, prev_path_delta = 0;

    // Variable for storing the main algorithm alg_state
    int alg_state = 0;

    // Variables for tracking key path points
    geometry_msgs::Point cur_pos, last_path_point, start_point, goal_point;

    // Variables for fixing the direction/position
    double cur_yaw = 0, desired_yaw = 0, err_yaw = 0;

    // Method for checking if the goal has been reached
    void check_if_goal_is_reached();

    // Methods for processing responses from Laser&Odometry
    void clbk_odom(const nav_msgs::Odometry::ConstPtr &msg);
    void clbk_laser(const sensor_msgs::LaserScan::ConstPtr &msg);

    // Method returns distance between two points
    double calc_dist_points(geometry_msgs::Point A, geometry_msgs::Point B);

    // Method for creating a create_report about algorithm performance
    void create_report(std::string msg);

    // Method for the angle normalising
    double normalize_angle(double angle);

    // Returns distance to obstacle in the direction of des_yaw
    double check_obstacle(double des_yaw);

    // Stops the robot
    void stop_robot();

    // Kills Gazebo and the algorithm
    void kill_system();

    // Method for going to the point
    void go_to_point(geometry_msgs::Point point);

    // Method for following the wall
    void wall_follower();

    // Method for monitoring indicators, such as sum_yaw and overall_distance
    void monitor_indicators();
};