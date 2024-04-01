#include "visbug21.h"

int main(int argc, char **argv) {
    // Initializing the VisBug21 algorithm with command line args
    ros::init(argc, argv, "visbug21");
    // Creating an instance of VisBug21
    VisBug21 visbug21;
    // Calling the main_logic() method to start the algorithm
    visbug21.main_logic();
    return EXIT_SUCCESS;
}

void VisBug21::show_points() {
    ROS_INFO_STREAM("Ti: x = " << Ti.x << " y = " << Ti.y);
    ROS_INFO_STREAM("Q: x = " << Q.x << " y = " << Q.y);
    ROS_INFO_STREAM("H: x = " << H.x << " y = " << H.y);
    ROS_INFO_STREAM("X: x = " << X.x << " y = " << X.y);
    ROS_INFO_STREAM("P: x = " << P.x << " y = " << P.y);
    ROS_INFO_STREAM("L: x = " << L.x << " y = " << L.y);
    // ROS_INFO_STREAM("S: x = " << S.x << " y = " << S.y);
    // ROS_INFO_STREAM("prev_H: x = " << prev_H.x << " y = " << prev_H.y);
}

void VisBug21::refresh() {
    // Variable for sleeping between iterations of ros::spinOnce()
    ros::Rate rate(RATE_FREQUENCY);
    // Updating the main loop
    ros::spinOnce();
    // Sleeping for 1/RATE_FREQUENCY seconds
    rate.sleep();
}

int VisBug21::radian2degree(double radian) {
    return round(radian * 180 / M_PI);
}

double VisBug21::degree2radian(int degree) {
    return (degree * M_PI / 180.0);
}

int VisBug21::adapt_degree(double interested_radian) {
    int angle = radian2degree(interested_radian) - radian2degree(cur_yaw);
    if (angle < 0) angle += 360;
    return angle;
}

double VisBug21::get_distance_by_course(double interested_radian) {
    int angle = adapt_degree(interested_radian);
    return regions[std::to_string(angle)];
}

void VisBug21::main_logic() {
    // Initially the robot goes to the goal (state 1 of the algorithm)
    change_state(1);
    while (nh.ok()) {
        show_points();
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
                go_to_point(Ti);
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
        refresh();
    }
}

char VisBug21::procedure_step_1() {
    ROS_INFO_STREAM("Compute Step 1");
    // Checking the target visibility
    if (goal_is_visible()) {
        Ti = goal_point;
        return 0;
    }
    // Checking if Ti is on an obstacle boundary
    if (point_is_on_boundary(Ti))
        return 3;
    // In all other cases going to step 2
    return 2;
}

char VisBug21::procedure_step_2() {
    ROS_INFO_STREAM("Compute Step 2");
    Q = search_endpoint_segment_Mline();
    Ti = Q;
    if (point_is_on_boundary(Q)) {
        ROS_INFO_STREAM("Hit point defined!");
        prev_H = H;
        H = Q;
        X = Q;
        return 3;
    }
    return 4;
}

char VisBug21::procedure_step_3() {
    ROS_INFO_STREAM("Compute Step 3");
    Q = search_endpoint_segment_boundary();
    if (segment_crosses_Mline(Ti, Q)) {
        P = search_intersection_point(start_point, goal_point, Ti, Q);
        if (calc_dist_points(P, goal_point) < calc_dist_points(H, goal_point)) {
            X = P;
            if (!segment_crosses_obstacle()) {
                L = P;
                Ti = P;
                return 2;
            }
        }
    }
    check_reachability();
    Ti = Q;
    return 4;
}

char VisBug21::procedure_step_4() {
    ROS_INFO_STREAM("Compute Step 4");
    Q = point_is_on_Mline(Ti) ? Ti : X;
    // S_apostrophe_point = search_endpoint_segment_Mline();
    // if (calc_dist_points(S_apostrophe_point, goal_point) < calc_dist_points(Q, goal_point) && is_in_main_semiplane()) {
    //     Ti = S_apostrophe_point;
    //     return 2;
    // }
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

bool VisBug21::point_is_on_boundary(Point point) {
    ROS_INFO_STREAM("Checking point is on boundary...");
    double dir_interest = atan2(point.y - cur_pos.y, point.x - cur_pos.x);
    ROS_INFO_STREAM("Degree interest for boundary: " << radian2degree(dir_interest));
    double math_dist = calc_dist_points(cur_pos, point);
    ROS_INFO_STREAM("Math dist = " << math_dist);
    ROS_INFO_STREAM("Distance by course = " << get_distance_by_course(dir_interest));
    if (fabs(get_distance_by_course(dir_interest) - math_dist) < DELTA_OBSTACLE_DECISION) {
        ROS_INFO_STREAM("On boundary!");
        return true;
    }
    ROS_INFO_STREAM("NOT boundary");
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
    // 360 regions by one degree
    for (int i = 0; i < 360; ++i)
        regions[std::to_string(i)] = std::min(msg->ranges[i], BASE_DIST);
}

// Add check for segment, not for the whole line?
Point VisBug21::math_search_endpoint_Mline() {
    Point point;
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
        double t = -B / (2 * A);
        point.x = goal_point.x + t * dx;
        point.y = goal_point.y + t * dy;
        return point;
    }

    double t = (double)((-B + sqrt(det)) / (2 * A));
    point.x = goal_point.x + t * dx;
    point.y = goal_point.y + t * dy;
    t = (double)((-B - sqrt(det)) / (2 * A));
    Point another;
    another.x = goal_point.x + t * dx;
    another.y = goal_point.y + t * dy;
    if (calc_dist_points(point, goal_point) < calc_dist_points(another, goal_point))
        return point;
    return another;

}

Point VisBug21::search_endpoint_segment_Mline() {
    ROS_INFO_STREAM("Entered cyclic");
    Point potential_Mline_point = math_search_endpoint_Mline();
    double dir_interest = atan2(potential_Mline_point.y - cur_pos.y, potential_Mline_point.x - cur_pos.x);
    while(calc_dist_points(cur_pos, potential_Mline_point) > get_distance_by_course(dir_interest)) {
        potential_Mline_point = math_search_endpoint_Mline();
        dir_interest = atan2(potential_Mline_point.y - cur_pos.y, potential_Mline_point.x - cur_pos.x);
        potential_Mline_point = move_along_Mline(potential_Mline_point);
        ROS_INFO_STREAM("Hypothetical Mline_point:\n" << potential_Mline_point);
        ROS_INFO_STREAM("DIR INTEREST = " << radian2degree(dir_interest));
        ROS_INFO_STREAM("Distance to Mline: " << get_distance_by_course(dir_interest));
        ROS_INFO_STREAM("Distance between cur_pos and potential Mline point: " << calc_dist_points(cur_pos, potential_Mline_point));
        show_points();
        refresh();
    }
    ROS_INFO_STREAM("Exited cyclic");
    return potential_Mline_point;
}

Point VisBug21::move_along_Mline(Point point) {
    double len = calc_dist_points(start_point, point);
    double step = len * SENSIVITY_MOVE_MLINE;
    double dx = (start_point.x - point.x) / len;
    double dy = (start_point.y - point.y) / len;
    point.x += dx * step;
    point.y += dy * step;
    return point;
}

Point VisBug21::search_endpoint_segment_boundary() {
    Point potential_Q_point;
    double dir_Ti = atan2(Ti.y - cur_pos.y, Ti.x - cur_pos.x);
    int robot_degree = adapt_degree(dir_Ti);
    int base_degree = radian2degree(dir_Ti);
    int potential_angle = base_degree;
    double dist_Q = 0;
    if (DIR_IS_LEFT) {
        for (int i = robot_degree; i - robot_degree < RANGE_SEARCH_DEGREES; ++i) {
            double cur_dist = regions[std::to_string(i > 360 ? i - 360 : i)];
            if (cur_dist < BASE_DIST) {
                dist_Q = cur_dist;
                potential_angle++;
            } else break;
        }
        if (potential_angle > 180) potential_angle -= 360;
    } else {
        for (int i = robot_degree; robot_degree - i < RANGE_SEARCH_DEGREES; --i) {
            double cur_dist = regions[std::to_string(i < 0 ? i + 360 : i)];
            if (cur_dist < BASE_DIST) {
                dist_Q = cur_dist;
                potential_angle--;
            } else break;
        }
        if (potential_angle < -180) potential_angle += 360;
    }
    potential_Q_point.x = cur_pos.x + cos(degree2radian(potential_angle)) * dist_Q;
    potential_Q_point.y = cur_pos.y + sin(degree2radian(potential_angle)) * dist_Q; 
    return potential_Q_point;
}

bool VisBug21::segment_crosses_obstacle() {
    Point test = search_intersection_point(P, goal_point, Ti, Q);
    if (fabs(test.x - P.x) < DELTA_SEGM_CR_OBST && fabs(test.y - P.y) < DELTA_SEGM_CR_OBST)
        return false;
    return true;
}

Point VisBug21::search_intersection_point(Point A, Point B, Point C, Point D) {
    Point result;
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

bool VisBug21::segment_crosses_Mline(Point A, Point B) {
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

bool VisBug21::point_is_on_Mline(Point point) {
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
    if (calc_dist_points(H, prev_H) < BUFFER_CHECK_REACHABILITY && H.x != 0 && H.y != 0 && prev_H.x != 0 && prev_H.y != 0) {
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
    double visible_dist = get_distance_by_course(atan2(goal_point.y - cur_pos.y, goal_point.x - cur_pos.x));
    if (geom_dist < VISION_RADIUS && visible_dist >= geom_dist) {
        ROS_INFO_STREAM("Goal is visible");
        return true;
    }
    return false;
}

bool VisBug21::cur_pos_is_Ti() {
    if (calc_dist_points(Ti, cur_pos) < ACCURACY_CUR_POS_IS_Ti) {
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