#include "visbug21.h"

int main(int argc, char **argv) {
    // Initializing the VisBug21 algorithm with command line args
    ros::init(argc, argv, "visbug21");
    // Creating an instance of Class1
    VisBug21 visbug21;
    // Calling the main_logic() method to start the algorithm
    visbug21.main_logic();
    return EXIT_SUCCESS;
}

void VisBug21::main_logic() {
    // Variable for sleeping between iterations of ros::spinOnce()
    ros::Rate rate(RATE_FREQUENCY);
    // Initially the robot goes to the goal (state 1 of the algorithm)
    change_state_alg(1);
    while (nh.ok()) {
        // Recording the change of yaw/position
        monitor_indicators();
        // Performing check of reaching the target
        check_if_goal_is_reached();
        // Computing procedure ComputeTi-21
        computeTi21();
        switch (alg_state) {
            case 1:
                // Moving towards Ti 
                go_to_point(Ti_pos);
                // Checking cur_pos = Ti
                if (cur_pos_is_Ti())
                    change_state_alg(2);
                break;
            case 2:
                // Moving along obstacle boundary
                wall_follower();
                // Checking cur_pos != Ti
                if (!cur_pos_is_Ti())
                    change_state_alg(1);
                break;
        }
        // Updating the main loop
        ros::spinOnce();
        // Sleeping for 1/RATE_FREQUENCY seconds
        rate.sleep();
    }
}

VisBug21::VisBug21() {
    sub_spec_distances_laser = nh.subscribe("/scan", 1, &VisBug21::clbk_spec_distances_laser, this);
}

void VisBug21::clbk_spec_distances_laser(const sensor_msgs::LaserScan::ConstPtr &msg) {
    yaw_goal = atan2(goal_point.y - cur_pos.y, goal_point.x - cur_pos.x);
    degree_goal = normalize_angle(yaw_goal - cur_yaw);
    if (degree_goal < 0) degree_goal += 360;
    regions["to_goal"] = std::min(msg->ranges[degree_goal], BASE_DIST);
}

void VisBug21::computeTi21() {
    switch (procedure_state) {
        // The last stage when target is visible
        case 1:
            // Checking the target visibility
            if (goal_is_visible())
                Ti_pos = goal_point;
            // Checking if Ti is on an obstacle boundary
            else if (point_is_on_boundary(Ti_pos))
                change_state_procedure(3);
            // In all other cases going to step 2
            else
                change_state_procedure(2);
            break;
        // Candidates for Ti along the M-line are processed && hit points are defined
        case 2:
            Q_pos = search_endpoint_segment_Mline();
            Ti_pos = Q_pos;
            if (point_is_on_boundary(Q_pos)) {
                prev_H_pos = H_pos;
                H_pos = Q_pos;
                X_pos = Q_pos;
                change_state_procedure(3);
            } else
                change_state_procedure(4);
            break;
        // Candidates for Ti along obstacle boundaries are processed && leave points are defined
        case 3:
            Q_pos = search_endpoint_segment_boundary();
            if (boundary_crosses_Mline()) {
                P_pos = search_boundary_Mline_intersection_point();
                if (calc_dist_points(P_pos, goal_point) < calc_dist_points(H_pos, goal_point)) {
                    X_pos = P_pos;
                    if (!segment_crosses_obstacle(P_pos, goal_point)) {
                        L_pos = P_pos;
                        Ti_pos = P_pos;
                        change_state_procedure(2);
                    }
                }
            } else {
                Ti_pos = Q_pos;
                change_state_procedure(4);
            }
            check_reachability();
            break;
        // Candidates for Ti - points of M-line noncontiguous to previous sets of points - are processed
        case 4:
            Q_pos = point_is_on_Mline(Ti_pos) ? Ti_pos : X_pos;
            S_apostrophe_point = search_closest_to_goal_Mline_point();
            if (calc_dist_points(S_apostrophe_point, goal_point) < calc_dist_points(Q_pos, goal_point) && is_in_main_semiplane()) {
                Ti_pos = S_apostrophe_point;
                change_state_procedure(2);
            } else
                change_state_procedure(1);
            break;
    }
}

bool VisBug21::point_is_on_Mline(geometry_msgs::Point point) {
    double A = start_point.y - goal_point.y;
    double B = goal_point.x - start_point.x;
    double C = start_point.x * goal_point.y - goal_point.x * start_point.y;
    double distance = fabs(A * point.x + B * point.y + C) / sqrt(pow(A, 2) + pow(B, 2));
    return distance <= ACCURACY_MLINE;
}

bool VisBug21::is_in_main_semiplane() {
    double A = goal_point.y - start_point.y;
    double B = start_point.x - goal_point.x;
    double C = goal_point.x * start_point.y - start_point.x * goal_point.y;
    double D = A * cur_pos.x + B * cur_pos.y + C;
    if ((D >= 0 && DIR_IS_LEFT) || (D <= 0 && !DIR_IS_LEFT))
        return true;
    return false;
}

void VisBug21::check_reachability() {
    if (calc_dist_points(H_pos, prev_H_pos) < BUFFER_CHECK_REACHABILITY) {
        stop_robot();
        create_report("Target can not be reached.");
        kill_system();
    }
}

bool VisBug21::goal_is_visible() {
    // Checking if free space it the direction of goal is less than vision radius
    if (regions["to_goal"] < VISION_RADIUS) {
        ROS_INFO_STREAM("Goal is visible");
        return true;
    }
    return false;
}

bool VisBug21::cur_pos_is_Ti() {
    if (calc_dist_points(Ti_pos, cur_pos) < ACCURACY_CUR_POS_IS_Ti)
        return true;
    return false;
}

void VisBug21::change_state_alg(int input_state) {
    alg_state = input_state;
    ROS_INFO_STREAM("Algorithm state changed: " << MAIN_STATE_NAMES[alg_state]);
}

void VisBug21::change_state_procedure(int input_state) {
    procedure_state = input_state;
    ROS_INFO_STREAM("Algorithm state changed: " << PROCEDURE_STATE_NAMES[procedure_state]);
}