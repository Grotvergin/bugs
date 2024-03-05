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
    degree_goal = normalize_angle(yaw_goal - cur_yaw) * 180 / M_PI;
    if (degree_goal < 0) degree_goal += 360;
    regions["to_goal"] = std::min(msg->ranges[degree_goal], BASE_DIST);
    yaw_endpoint_Mline = atan2(potential_Mline_point.y - cur_pos.y, potential_Mline_point.x - cur_pos.x);
    degree_endpoint_Mline = normalize_angle(yaw_endpoint_Mline - cur_yaw) * 180 / M_PI;
    if (degree_endpoint_Mline < 0) degree_endpoint_Mline += 360;
    regions["to_Mline"] = std::min(msg->ranges[degree_endpoint_Mline], BASE_DIST);
    regions["to_boundary"] = std::min(msg->ranges[search_angle_endpoint_segment_boundary(msg)], BASE_DIST);
}

double VisBug21::search_angle_endpoint_segment_boundary(const sensor_msgs::LaserScan::ConstPtr &msg) {
    double max_dist = 0;
    double des_angle = 0;
    for (int i = 0; i <= 360; ++i) {
        if (msg->ranges[i] > max_dist && msg->ranges[i] <= VISION_RADIUS) {
            max_dist = msg->ranges[i];
            des_angle = i;
        }
    }
    return des_angle;
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
            if (segment_crosses_Mline(Ti_pos, Q_pos)) {
                P_pos = search_intersection_point(start_point, goal_point, Ti_pos, Q_pos);
                if (calc_dist_points(P_pos, goal_point) < calc_dist_points(H_pos, goal_point)) {
                    X_pos = P_pos;
                    if (segment_not_crosses_obstacle(P_pos, goal_point)) {
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
            S_apostrophe_point = search_endpoint_segment_Mline();
            if (calc_dist_points(S_apostrophe_point, goal_point) < calc_dist_points(Q_pos, goal_point) && is_in_main_semiplane()) {
                Ti_pos = S_apostrophe_point;
                change_state_procedure(2);
            } else
                change_state_procedure(1);
            break;
    }
}

geometry_msgs::Point VisBug21::search_endpoint_segment_boundary() {
    
}

bool VisBug21::segment_not_crosses_obstacle(geometry_msgs::Point A, geometry_msgs::Point B) {
    geometry_msgs::Point test = search_intersection_point(P_pos, goal_point, Ti_pos, Q_pos);
    if (test.x != P_pos.x && test.y != P_pos.y);
        return true;
    return false;
}

geometry_msgs::Point VisBug21::search_intersection_point(geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C, geometry_msgs::Point D) {
    geometry_msgs::Point result;
    result.x = result.y = result.z = 0;
    double denom = (D.y - C.y) * (B.x - A.x) - (D.x - C.x) * (B.y - A.y);
    if (denom == 0)
        return result;
    double ua = ((D.x - C.x) * (A.y - C.y) - (D.y - C.y) * (A.x - C.x)) / denom;
    if (ua < 0 || ua > 1)
        return result;
    double ub = ((B.x - A.x) * (A.y - C.y) - (B.y - A.y) * (A.x - C.x)) / denom;
    if (ub < 0 || ub > 1)
        return result;
    result.x = A.x + ua * (B.x - A.x);
    result.y = A.y + ua * (B.y - A.y);
    return result;
}

bool VisBug21::is_between(double x, double b1, double b2) {
    return (((x >= (b1 - ACCURACY_LINES)) && (x <= (b2 + ACCURACY_LINES))) || ((x >= (b2 - ACCURACY_LINES)) && (x <= (b1 + ACCURACY_LINES))));
}

bool VisBug21::segment_crosses_Mline(geometry_msgs::Point A, geometry_msgs::Point B) {
    double dx1 = B.x - A.x;
    double dx2 = goal_point.x - start_point.x;
    double dy1 = B.y - A.y;
    double dy2 = goal_point.y - start_point.y;

    if (abs(dx1) < ACCURACY_LINES && abs(dx2) < ACCURACY_LINES)
        return false;
    if (abs((dy1 / dx1) - (dy2 / dx2)) < ACCURACY_LINES)
        return false;

    double xcol = ((dx1 * dx2) * (start_point.y - A.y) - start_point.x * dy2 * dx1 + A.x * dy1 * dx2) / (dy1 * dx2 - dy2 * dx1);
    double ycol = 0;
    if (dx1 < ACCURACY_LINES)
        ycol = ((xcol * dy2) + (start_point.y * dx2) - (start_point.x * dy2)) / dx2;
    else
        ycol = ((xcol * dy1) + (A.y * dx1) - (A.x * dy1)) / dx1;

    return is_between(xcol, A.x, B.x) && is_between(ycol, A.y, B.y) && is_between(xcol, start_point.x, goal_point.x) && is_between(ycol, start_point.y, goal_point.y);
}

geometry_msgs::Point VisBug21::math_search_endpoint_Mline() {
    geometry_msgs::Point first, second;
    // Calculating the slope
    double k = (goal_point.y - start_point.y) / (goal_point.x - start_point.x);
    // Calculating the y-axis offset
    double b = start_point.y - k * start_point.x;

    double a = 1 + k * k;
    double b_circle = 2 * k * (b - cur_pos.y) - 2 * cur_pos.x;
    double c = pow(cur_pos.x, 2) + pow((b - cur_pos.y), 2) + pow(VISION_RADIUS, 2);

    double D = pow(b_circle, 2) - 4 * a * c;
    if (D < 0) {
        ROS_INFO_STREAM("No intersections found");
        first.x = first.y = first.z = 0;
        return first;
    }

    first.x = (-b_circle + sqrt(D)) / (2 * a);
    second.x = (-b_circle - sqrt(D)) / (2 * a);
    first.y = k * first.x + b;
    second.y = k * second.x + b;

    double dist_first = sqrt(pow(first.x - goal_point.x, 2) + pow(first.y - goal_point.y, 2));
    double dist_second = sqrt(pow(second.x - goal_point.x, 2) + pow(second.y - goal_point.y, 2));

    if (dist_first <= dist_second)
        return first;
    return second;
}

geometry_msgs::Point VisBug21::search_endpoint_segment_Mline() {
    potential_Mline_point = math_search_endpoint_Mline();
    while(calc_dist_points(cur_pos, potential_Mline_point) > regions["to_Mline"])
        move_along_Mline();
    return potential_Mline_point;
}

void VisBug21::move_along_Mline() {
    double l = sqrt(pow(goal_point.x - start_point.x, 2) + pow(goal_point.y - start_point.y, 2));
    double t = STEP_MLINE / l;
    potential_Mline_point.x = start_point.x + t * (goal_point.x - start_point.x);
    potential_Mline_point.y = start_point.y + t * (goal_point.y - start_point.y);
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