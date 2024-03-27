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
    change_state(1);
    while (nh.ok()) {
        // Recording the change of yaw/position
        monitor_indicators();
        // Performing check of reaching the target
        check_if_goal_is_reached();
        // Computing procedure ComputeTi-21
        computeTi21();
        ROS_INFO_STREAM("Starting main body");
        switch (state) {
            case 1:
                // Moving towards Ti 
                go_to_point(Ti_pos);
                // Checking cur_pos = Ti
                if (cur_pos_is_Ti())
                    change_state(2);
                break;
            case 2:
                // Moving along obstacle boundary
                wall_follower();
                // Checking cur_pos != Ti
                if (!cur_pos_is_Ti())
                    change_state(1);
                break;
        }
        // Updating the main loop
        ros::spinOnce();
        // Sleeping for 1/RATE_FREQUENCY seconds
        rate.sleep();
    }
}

char VisBug21::procedure_step_1() {
    ROS_INFO_STREAM("Compute Step 1");
    // Checking the target visibility
    if (goal_is_visible()) {
        Ti_pos = goal_point;
        return 0;
    }
    // Checking if Ti is on an obstacle boundary
    if (point_is_on_boundary(Ti_pos))
        return 3;
    // In all other cases going to step 2
    return 2;
}

char VisBug21::procedure_step_2() {
    ROS_INFO_STREAM("Compute Step 2");
    Q_pos = search_endpoint_segment_Mline();
    Ti_pos = Q_pos;
    ROS_INFO_STREAM("Ti = Q_pos = " << Ti_pos);
    if (point_is_on_boundary(Q_pos)) {
        prev_H_pos = H_pos;
        H_pos = Q_pos;
        X_pos = Q_pos;
        return 3;
    }
    return 4;
}

char VisBug21::procedure_step_3() {
    ROS_INFO_STREAM("Compute Step 3");
    Q_pos = search_endpoint_segment_boundary();
    check_reachability();
    if (segment_crosses_Mline(Ti_pos, Q_pos)) {
        P_pos = search_intersection_point(start_point, goal_point, Ti_pos, Q_pos);
        if (calc_dist_points(P_pos, goal_point) < calc_dist_points(H_pos, goal_point)) {
            X_pos = P_pos;
            if (segment_not_crosses_obstacle(P_pos, goal_point)) {
                L_pos = P_pos;
                Ti_pos = P_pos;
                return 2;
            }
        }
    } else {
        Ti_pos = Q_pos;
        return 4;
    }
}

char VisBug21::procedure_step_4() {
    ROS_INFO_STREAM("Compute Step 4");
    Q_pos = point_is_on_Mline(Ti_pos) ? Ti_pos : X_pos;
    S_apostrophe_point = search_endpoint_segment_Mline();
    if (calc_dist_points(S_apostrophe_point, goal_point) < calc_dist_points(Q_pos, goal_point) && is_in_main_semiplane()) {
        Ti_pos = S_apostrophe_point;
        return 2;
    }
    return 0;
}

void VisBug21::computeTi21() {
    ROS_INFO_STREAM("Entered Compute");
    char next_step = procedure_step_1();
    while(next_step) {
        if (next_step == 2)
            next_step = procedure_step_2();
        else if (next_step == 3)
            next_step = procedure_step_3();
        else next_step = procedure_step_4();
    }
    ROS_INFO_STREAM("Exited Compute");
}

bool VisBug21::point_is_on_boundary(geometry_msgs::Point point) {
    double needed_yaw = atan2(point.y - cur_pos.y, point.x - cur_pos.x);
    double degree = normalize_angle(needed_yaw - cur_yaw);
    degree = degree * 180 / M_PI;
    if (degree < 0)
        degree += 360;
    angle_for_decision = degree;
    double math_dist = calc_dist_points(cur_pos, point);
    ROS_INFO_STREAM("Boundary decision math_Dist = " << math_dist);
    ROS_INFO_STREAM("Boudary decision real_dist = " << regions["to_unknown"]);
    if (fabs(regions["to_unknown"] - math_dist) < DELTA_OBSTACLE_DECISION && !cur_pos_is_Ti())
        return true;
    return false;
}

void VisBug21::clbk_laser(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // Base regions
    regions["left"] = std::min(*std::min_element(msg->ranges.begin() + 57, msg->ranges.begin() + 102), BASE_DIST);
    regions["fleft"] = std::min(*std::min_element(msg->ranges.begin() + 26, msg->ranges.begin() + 56), BASE_DIST);
    regions["front"] = std::min(std::min(*std::min_element(msg->ranges.begin(), msg->ranges.begin() + 25), *std::min_element(msg->ranges.begin() + 334, msg->ranges.begin() + 359)), BASE_DIST);
    regions["fright"] = std::min(*std::min_element(msg->ranges.begin() + 303, msg->ranges.begin() + 333), BASE_DIST);
    regions["right"] = std::min(*std::min_element(msg->ranges.begin() + 257, msg->ranges.begin() + 302), BASE_DIST);
    regions["right45"] = std::min(msg->ranges[315], BASE_DIST);
    regions["left45"] = std::min(msg->ranges[45], BASE_DIST);
    // Special regions
    yaw_endpoint_Mline = atan2(potential_Mline_point.y - cur_pos.y, potential_Mline_point.x - cur_pos.x);
    degree_endpoint_Mline = normalize_angle(yaw_endpoint_Mline - cur_yaw) * 180 / M_PI;
    if (degree_endpoint_Mline < 0) degree_endpoint_Mline += 360;
    ROS_INFO_STREAM("Degree endpoint Mline: " << degree_endpoint_Mline);
    regions["to_Mline"] = std::min(msg->ranges[degree_endpoint_Mline], VISION_RADIUS);
    yaw_goal = atan2(goal_point.y - cur_pos.y, goal_point.x - cur_pos.x);
    degree_goal = normalize_angle(yaw_goal - cur_yaw) * 180 / M_PI;
    if (degree_goal < 0) degree_goal += 360;
    regions["to_goal"] = std::min(msg->ranges[degree_goal], VISION_RADIUS);
    angle_to_boundary = search_angle_endpoint_segment_boundary(msg);
    regions["to_boundary"] = std::min(msg->ranges[angle_to_boundary], VISION_RADIUS);
    regions["to_unknown"] = std::min(msg->ranges[angle_for_decision], VISION_RADIUS);
}

// Add check for segment, not for the hole line?
geometry_msgs::Point VisBug21::math_search_endpoint_Mline() {
    geometry_msgs::Point point;
    double dx = start_point.x - goal_point.x;
    double dy = start_point.y - goal_point.y;

    double A = dx * dx + dy * dy;
    double B = 2 * (dx * (goal_point.x - cur_pos.x) + dy * (goal_point.y - cur_pos.y));
    double C = pow((goal_point.x - cur_pos.x), 2) + pow((goal_point.y - cur_pos.y), 2) - pow(VISION_RADIUS, 2);

    double det = B * B - 4 * A * C;

    if (A <= 0.0000001 || det < 0) {
        ROS_INFO_STREAM("No mathematical intersection with Mline found");
        return point;
    }

    if (fabs(det) < 0.0000001) {
        ROS_INFO_STREAM("One mathematical intersection with Mline found");
        double t = -B / (2 * A);
        point.x = goal_point.x + t * dx;
        point.y = goal_point.y + t * dy;
        return point;
    }

    ROS_INFO_STREAM("Two mathematical intersections with Mline found");
    double t = (double)((-B + sqrt(det)) / (2 * A));
    point.x = goal_point.x + t * dx;
    point.y = goal_point.y + t * dy;
    t = (double)((-B - sqrt(det)) / (2 * A));
    geometry_msgs::Point another;
    another.x = goal_point.x + t * dx;
    another.y = goal_point.y + t * dy;
    if (calc_dist_points(point, goal_point) < calc_dist_points(another, goal_point))
        return point;
    return another;

}

geometry_msgs::Point VisBug21::search_endpoint_segment_Mline() {
    potential_Mline_point = math_search_endpoint_Mline();
    ROS_INFO_STREAM("Hypothetical Mline_point:\n" << potential_Mline_point);
    // REMOVED BOUNDARY CHECK FOR SIMPLICITY
    // while(calc_dist_points(cur_pos, potential_Mline_point) > regions["to_Mline"] && regions["to_Mline"] > 0) {
    //     ROS_INFO_STREAM("Distance regions[Mline]: " << regions["to_Mline"]);
    //     ROS_INFO_STREAM("Distance between cur_pos and potential Mline point: " << calc_dist_points(cur_pos, potential_Mline_point));
    //     potential_Mline_point = move_along_Mline(potential_Mline_point);
    // }
    return potential_Mline_point;
}

// FUNCTION DOES NOT WORK AT ALL
geometry_msgs::Point VisBug21::move_along_Mline(geometry_msgs::Point point) {
    double l = sqrt(pow(goal_point.x - start_point.x, 2) + pow(goal_point.y - start_point.y, 2));
    double t = STEP_MLINE / l;
    point.x = start_point.x + t * (goal_point.x - start_point.x);
    point.y = start_point.y + t * (goal_point.y - start_point.y);
    return point;
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

geometry_msgs::Point VisBug21::search_endpoint_segment_boundary() {
    double angle_x = cur_yaw * 180 / M_PI;
    double normalised_boundary_angle = 0;
    if (angle_to_boundary > 180)
        normalised_boundary_angle = 360 - angle_to_boundary;
    else
        normalised_boundary_angle = -angle_to_boundary;
    Q_pos.x = cur_pos.x + (regions["to_boundary"] * cos(normalised_boundary_angle - angle_x));
    Q_pos.y = cur_pos.y + (regions["to_boundary"] * sin (normalised_boundary_angle - angle_x));
    return Q_pos;
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
    if (calc_dist_points(H_pos, prev_H_pos) < BUFFER_CHECK_REACHABILITY && H_pos.x != 0 && H_pos.y != 0 && prev_H_pos.x != 0 && prev_H_pos.y != 0) {
        stop_robot();
        create_report("Target can not be reached.");
        kill_system();
    }
}

bool VisBug21::goal_is_visible() {
    // Checking two conditions:
    // 1. Geometrical distance between goal and current is less than vision radius
    // 2. Free distance to goal is more than geometrical
    double geom_dist = calc_dist_points(cur_pos, goal_point);
    if (geom_dist < VISION_RADIUS && regions["to_goal"] >= geom_dist) {
        ROS_INFO_STREAM("Goal is visible");
        return true;
    }
    return false;
}

bool VisBug21::cur_pos_is_Ti() {
    if (calc_dist_points(Ti_pos, cur_pos) < ACCURACY_CUR_POS_IS_Ti) {
        ROS_INFO_STREAM("Current position is Ti");
        return true;
    }
    return false;
}

void VisBug21::change_state(int input_state) {
    state = input_state;
    if (input_state == 1)
        ROS_INFO_STREAM("Main Step 1");
    else ROS_INFO_STREAM("Main Step 2");
}