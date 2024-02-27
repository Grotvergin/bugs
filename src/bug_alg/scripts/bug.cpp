#include "bug.h"

BugAlg::BugAlg() {
    // Subsribing & advertising some ROS topics
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    sub_laser = nh.subscribe("/scan", 1, &BugAlg::clbk_laser, this);
    sub_odom = nh.subscribe("/odom", 1, &BugAlg::clbk_odom, this);

    // Waiting for Gazebo activation & subsribing
    ros::service::waitForService("/gazebo/set_model_state");
    reset_world = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    srv_client_set_model_state = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    reset_world = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

    // Initializing start & goal points
    ros::param::get("initial_x", start_point.x);
    ros::param::get("initial_y", start_point.y);
    ros::param::get("des_pos_x", goal_point.x);
    ros::param::get("des_pos_y", goal_point.y);
    start_point.z = goal_point.z = 0;

    // Initializing last_path_point as the start point, prev_yaw as cur_yaw
    last_path_point = start_point;
    prev_yaw = cur_yaw;

    // Logging information about major points
    ROS_INFO_STREAM("Initial point X: " << start_point.x << " Y: " << start_point.y << std::endl);
    ROS_INFO_STREAM("Destination point X: " << goal_point.x << " Y: " << goal_point.y << std::endl);
}

void BugAlg::monitor_indicators() {
    // Updating the overall rotation
    sum_yaw += fabs(cur_yaw - prev_yaw);
    // Previous yaw is now current yaw
    prev_yaw = cur_yaw;
    // Updating the previous small segment of path
    prev_path_delta = calc_dist_points(last_path_point, cur_pos);
    // Updating the overall distance by the value of previous segment
    overall_distance += prev_path_delta;
    // Previous path point is now current position
    last_path_point = cur_pos;
}

void BugAlg::check_if_goal_is_reached() {
    // Checking the distance to the goal
    if (calc_dist_points(goal_point, cur_pos) < ACCURACY_TARGET) {
        // Finishing the algorithm execution
        stop_robot();
        // Creating a report about the reached target
        create_report("Point is reached.");
        // Killing the system and processes
        kill_system();
    }
}

void BugAlg::create_report(std::string msg) {
    // Logging the final message
    ROS_INFO_STREAM(msg);
    // Trying to create a file for report
    std::ofstream file(RESULT_FILE_NAME);
    if (file.is_open()) {
        // Writing some indicators to the file
        file << "Elapsed time: " << (std::chrono::system_clock::now() - start_time).count();
        file << " nanoseconds" << std::endl;
        file << "Hit points (obstacles) encountered: " << std::to_string(obstacle_count) << std::endl;
        file << "Distance travelled: " << std::to_string(overall_distance) << std::endl;
        // Structure is created for memory management
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        file << "Memory usage: " << std::to_string(usage.ru_maxrss) << " Kb" << std::endl;
        // Subjective authors' complexity assessment
        file << "Algorithm complexity: " << ALG_COMPLEXITY << std::endl;
        file << "Calculation time: ?" << std::endl;
        file << "Total turn: " << std::to_string(sum_yaw) << std::endl;
        file.close();
    } else {
        ROS_ERROR_STREAM("File can not be opened!");
    }    
}

void BugAlg::kill_system() {
    // Resetting the Gazebo world
    std_srvs::Empty srv;
    reset_world.call(srv);
    // Killing the process of current algorithm
    std::string str = "kill " + std::to_string(getpid());
    system(str.c_str());
}

void BugAlg::go_to_point(geometry_msgs::Point point) {
    // Creating an empty twist message
    geometry_msgs::Twist msg;
    // Calculating the desired angle to the target
    desired_yaw = atan2(point.y - cur_pos.y, point.x - cur_pos.x);
    // Calculating the error, which needs to be fixed
    err_yaw = normalize_angle(desired_yaw - cur_yaw);
    if (fabs(err_yaw) > YAW_PRECISION) {
        // If there is a huge error, robot stops and rotates to the right direction
        msg.linear.x = 0;
        msg.angular.z = err_yaw > 0 ? ROTATE_SPEED : -ROTATE_SPEED;
    } else {
        // If error is not huge, robot just goes straight with no rotation
        msg.linear.x = NORMAL_SPEED_FORWARD;
        msg.angular.z = 0;
    }
    // Publishing the twist message
    pub.publish(msg);
}

void BugAlg::wall_follower() {
    // Creating an empty twist message
    geometry_msgs::Twist msg;
    // String which contains current case of the relative position of wall and robot
    std::string state_description = "";
    // Choosing the side to look at
    std::string dir_to_check = DIR_IS_LEFT ? "right45" : "left45";
    // Block of conditions on different relative positions
    // Robot is in front of the wall
    if (regions["front"] < LASER_RANGE && regions["fleft"] > LASER_RANGE && regions["fright"] > LASER_RANGE) {
        state_description = "Front";
        msg.angular.z = DIR_IS_LEFT ? ROTATE_SPEED : -ROTATE_SPEED;
    // The wall is in front-right from the robot
    } else if (regions["front"] > LASER_RANGE && regions["fleft"] > LASER_RANGE && regions["fright"] < LASER_RANGE) {
        state_description = "Front-right";
        msg.linear.x = SLOW_SPEED_FORWARD;
    // Angle is correct, a little turn
    } else if (regions["front"] > LASER_RANGE && regions["fleft"] > LASER_RANGE && regions["fright"] > LASER_RANGE && regions[dir_to_check] > DIAG_RANGE) {
        state_description = "Correct angle";
        msg.angular.z = DIR_IS_LEFT ? -ROTATE_SPEED : ROTATE_SPEED;
    // The wall is in front-left from the robot
    } else if (regions["front"] > LASER_RANGE && regions["fleft"] < LASER_RANGE && regions["fright"] > LASER_RANGE) {
        state_description = "Front-left";
        msg.linear.x = SLOW_SPEED_FORWARD;
    // The wall is both detected in front and front-right regions
    } else if (regions["front"] < LASER_RANGE && regions["fleft"] > LASER_RANGE && regions["fright"] < LASER_RANGE) {
        state_description = "Front and front-right";
        msg.angular.z = DIR_IS_LEFT ? ROTATE_SPEED : -ROTATE_SPEED;
    // The wall is both detected in front and front-left regions
    } else if (regions["front"] < LASER_RANGE && regions["fleft"] < LASER_RANGE && regions["fright"] > LASER_RANGE) {
        state_description = "Front and front-left";
        msg.angular.z = DIR_IS_LEFT ? ROTATE_SPEED : -ROTATE_SPEED;
    // The wall is seen thrice: front-left, front and front-right
    } else if (regions["front"] < LASER_RANGE && regions["fleft"] < LASER_RANGE && regions["fright"] < LASER_RANGE) {
        state_description = "Front-left, front and front-right";
        msg.angular.z = DIR_IS_LEFT ? ROTATE_SPEED : -ROTATE_SPEED;
    // The wall is seen both by front-right and front-left, but not front
    } else if (regions["front"] > LASER_RANGE && regions["fleft"] < LASER_RANGE && regions["fright"] < LASER_RANGE) {
        state_description = "Front-left and front-right";
        msg.linear.x = SLOW_SPEED_FORWARD;
    } else {
        state_description = "Unknown situation";
    }
    ROS_INFO_STREAM(state_description);
    pub.publish(msg);
}

void BugAlg::stop_robot() {
    // Creating an empty twist message
    geometry_msgs::Twist twist_msg;
    // Assigning both angular and lineat velocities to zero
    twist_msg.linear.x = 0;
    twist_msg.angular.z = 0;
    // Publishing the twist message
    pub.publish(twist_msg);
}

double BugAlg::normalize_angle(double angle) {
    // If the angle is too big, we need to decrease it
    if (fabs(angle) > M_PI)
        angle -= (2 * M_PI * angle) / (fabs(angle));
    return angle;
}

void BugAlg::clbk_laser(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // Block of so-called regions, key areas, which the robot uses for orientation
    // In each direction we choose the smallest distance overall
    regions["left"] = std::min(*std::min_element(msg->ranges.begin() + 57, msg->ranges.begin() + 102), BASE_DIST);
    regions["fleft"] = std::min(*std::min_element(msg->ranges.begin() + 26, msg->ranges.begin() + 56), BASE_DIST);
    regions["front"] = std::min(std::min(*std::min_element(msg->ranges.begin(), msg->ranges.begin() + 25), *std::min_element(msg->ranges.begin() + 334, msg->ranges.begin() + 359)), BASE_DIST);
    regions["fright"] = std::min(*std::min_element(msg->ranges.begin() + 303, msg->ranges.begin() + 333), BASE_DIST);
    regions["right"] = std::min(*std::min_element(msg->ranges.begin() + 257, msg->ranges.begin() + 302), BASE_DIST);
    regions["right45"] = std::min(msg->ranges[315], BASE_DIST);
    regions["left45"] = std::min(msg->ranges[45], BASE_DIST);
}

void BugAlg::clbk_odom(const nav_msgs::Odometry::ConstPtr &msg) {
    // Receiving the odometry message
    cur_pos = msg->pose.pose.position;
    // Decompose the message into components
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    // Creating a matrix from the quaternion
    tf::Matrix3x3 m(q);
    // Roll and pitch are not used, but we still need to create variables
    double roll, pitch;
    // Created variables are required in the following method
    m.getRPY(roll, pitch, cur_yaw);
}

double BugAlg::calc_dist_points(geometry_msgs::Point A, geometry_msgs::Point B) {
    // Calculating the catheters
    double dist_y = A.y - B.y;
    double dist_x = A.x - B.x;
    // Applying the formula for the distance between two points
    return sqrt(pow(dist_y, 2) + pow(dist_x, 2));
}

double BugAlg::check_obstacle(double des_yaw) {
    // Creating an empty string for the chosen direction
    std::string dir;
    // Normalizing the desired yaw
    des_yaw = normalize_angle((des_yaw - cur_yaw) * 180 / M_PI);
    if (des_yaw < 0) des_yaw += 2 * M_PI;
    // Block of conditions for different desired yaws
    if(des_yaw > 57 && des_yaw < 102)
        dir = "left";
    else if(des_yaw > 26 && des_yaw < 56)
        dir = "fleft";
    else if((des_yaw > 0 && des_yaw < 25) || (des_yaw > 334 && des_yaw < 364)) 
        dir = "front";
    else if(des_yaw > 303 && des_yaw < 333)
        dir = "fright";
    else if(des_yaw > 257 && des_yaw < 302)
        dir = "right";
    // Returning current closest distance in the desired direction
    return regions[dir];
}