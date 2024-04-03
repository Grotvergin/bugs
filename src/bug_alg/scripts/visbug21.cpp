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
        switch (state) {
            case 1:
                // Moving towards Ti 
                go_to_point(Ti);
                // Checking cur_pos = Ti
                if ((cur_pos_is_Ti() || near_obstacle()) && !leaving_obstacle)
                    change_state(2);
                break;
            case 2:
                // Moving along obstacle boundary
                wall_follower();
                // Checking cur_pos != Ti
                if (!cur_pos_is_Ti() && !near_obstacle() || leaving_obstacle)
                    change_state(1);
                break;
        }
        refresh();
    }
}

void VisBug21::move_Ti(Point hypo_Ti) {
    if (calc_dist_points(hypo_Ti, goal_point) < calc_dist_points(Ti, goal_point))
        Ti = hypo_Ti;
}

void VisBug21::show_points() {
    ROS_INFO_STREAM("Ti: x = " << Ti.x << " y = " << Ti.y);
    ROS_INFO_STREAM("Q: x = " << Q.x << " y = " << Q.y);
    ROS_INFO_STREAM("H: x = " << H.x << " y = " << H.y);
    // ROS_INFO_STREAM("X: x = " << X.x << " y = " << X.y);
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

bool VisBug21::near_obstacle() {
    if (regions["front"] < DISTANCE_TO_OBSTACLE || regions["fright"] < DISTANCE_TO_OBSTACLE
    || regions["fleft"] < DISTANCE_TO_OBSTACLE || regions["right"] < DISTANCE_TO_OBSTACLE ||
    regions["left"] < DISTANCE_TO_OBSTACLE)
        return true;
    return false;
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
    move_Ti(Q);
    if (point_is_on_boundary(Q) && state == 1) {
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
    Point hypo_P = search_intersection_point(start_point, goal_point, Ti, Q);
    ROS_INFO_STREAM("Hypo P: x = " << hypo_P.x << " y = " << hypo_P.y);
    show_points();
    if ((hypo_P.x * hypo_P.y != 0 || point_is_on_Mline(Q)) && state == 2) {
        ROS_INFO_STREAM("Passed first check");
        if (hypo_P.x * hypo_P.y != 0)
            P = hypo_P;
        else if (point_is_on_Mline(Q))
            P = Q;
        show_points();
        if (calc_dist_points(P, goal_point) < calc_dist_points(H, goal_point)) {
            ROS_INFO_STREAM("calc_dist_points(P, goal_point) < calc_dist_points(H, goal_point)");
            X = P;
            show_points();
            if (enough_space_to_leave()) {
                ROS_INFO_STREAM("Leave point defined!");
                L = P;
                leaving_obstacle = true;
                Ti = goal_point;
                ROS_INFO_STREAM("FOUND Ti = " << math_search_endpoint_Mline());
                show_points();
                return 2;
            }
        }
    }
    // check_reachability();
    move_Ti(Q);
    return 4;
}

char VisBug21::procedure_step_4() {
    // ROS_INFO_STREAM("Compute Step 4");
    // Q = point_is_on_Mline(Ti) ? Ti : X;
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
    // ROS_INFO_STREAM("Checking point is on boundary...");
    double dir_interest = atan2(point.y - cur_pos.y, point.x - cur_pos.x);
    // ROS_INFO_STREAM("Degree interest for boundary: " << radian2degree(dir_interest));
    double math_dist = calc_dist_points(cur_pos, point);
    // ROS_INFO_STREAM("Math dist = " << math_dist);
    // ROS_INFO_STREAM("Distance by course = " << get_distance_by_course(dir_interest));
    // ROS_INFO_STREAM("Distance by course lower= " << get_distance_by_course(dir_interest + SECTOR_OBSTACLE_DECISION));
    // ROS_INFO_STREAM("Distance by course upper= " << get_distance_by_course(dir_interest - SECTOR_OBSTACLE_DECISION));
    // May be переполение градусов!!!!!!!
    if (fabs(get_distance_by_course(dir_interest) - math_dist) < DELTA_OBSTACLE_DECISION
    || fabs(get_distance_by_course(dir_interest + SECTOR_OBSTACLE_DECISION) - math_dist) < DELTA_OBSTACLE_DECISION
    || fabs(get_distance_by_course(dir_interest - SECTOR_OBSTACLE_DECISION) - math_dist) < DELTA_OBSTACLE_DECISION) {
        ROS_INFO_STREAM("On boundary!");
        return true;
    }
    ROS_INFO_STREAM("NOT boundary");
    return false;
}

void VisBug21::clbk_laser(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // Base regions
    regions["left"] = std::min(*std::min_element(msg->ranges.begin() + 57, msg->ranges.begin() + 102), VISION_RADIUS);
    regions["fleft"] = std::min(*std::min_element(msg->ranges.begin() + 26, msg->ranges.begin() + 56), VISION_RADIUS);
    regions["front"] = std::min(std::min(*std::min_element(msg->ranges.begin(), msg->ranges.begin() + 25), *std::min_element(msg->ranges.begin() + 334, msg->ranges.begin() + 359)), VISION_RADIUS);
    regions["fright"] = std::min(*std::min_element(msg->ranges.begin() + 303, msg->ranges.begin() + 333), VISION_RADIUS);
    regions["right"] = std::min(*std::min_element(msg->ranges.begin() + 257, msg->ranges.begin() + 302), VISION_RADIUS);
    regions["right45"] = std::min(msg->ranges[315], VISION_RADIUS);
    regions["left45"] = std::min(msg->ranges[45], VISION_RADIUS);
    // 360 regions by one degree
    for (int i = 0; i < 360; ++i)
        regions[std::to_string(i)] = std::min(msg->ranges[i], VISION_RADIUS);
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
        // ROS_INFO_STREAM("No mathematical intersection with Mline found");
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
        // ROS_INFO_STREAM("Hypothetical Mline_point:\n" << potential_Mline_point);
        // ROS_INFO_STREAM("DIR INTEREST = " << radian2degree(dir_interest));
        // ROS_INFO_STREAM("Distance to Mline: " << get_distance_by_course(dir_interest));
        // ROS_INFO_STREAM("Distance between cur_pos and potential Mline point: " << calc_dist_points(cur_pos, potential_Mline_point));
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
    // ROS_INFO_STREAM("Robot degree = " << robot_degree);
    int base_degree = radian2degree(dir_Ti);
    // ROS_INFO_STREAM("Base degree = " << base_degree);
    int potential_angle = base_degree;
    double dist_Q = calc_dist_points(cur_pos, Ti);
    if (DIR_IS_LEFT) {
        for (int i = robot_degree; i - robot_degree < RANGE_SEARCH_DEGREES; ++i) {
            double cur_dist = regions[std::to_string(i >= 360 ? i - 360 : i)];
            // ROS_INFO_STREAM("Loop i = " << i << " cur_dist = " << cur_dist);
            if (cur_dist < VISION_RADIUS) {
                dist_Q = cur_dist;
                potential_angle++;
            } else break;
        }
        if (potential_angle > 180) potential_angle -= 360;
    } else {
        for (int i = robot_degree; robot_degree - i < RANGE_SEARCH_DEGREES; --i) {
            double cur_dist = regions[std::to_string(i < 0 ? i + 360 : i)];
            if (cur_dist < VISION_RADIUS) {
                dist_Q = cur_dist;
                potential_angle--;
            } else break;
        }
        if (potential_angle < -180) potential_angle += 360;
    }
    // ROS_INFO_STREAM("Dist Q = " << dist_Q);
    // ROS_INFO_STREAM("Potential angle = " << potential_angle);
    potential_Q_point.x = cur_pos.x + cos(degree2radian(potential_angle)) * dist_Q;
    potential_Q_point.y = cur_pos.y + sin(degree2radian(potential_angle)) * dist_Q; 
    return potential_Q_point;
}

bool VisBug21::enough_space_to_leave() {
    double dir = atan2(goal_point.y - cur_pos.y, goal_point.x - cur_pos.x);
    double dist = get_distance_by_course(dir);
    // ROS_INFO_STREAM("Dir_leave = " << dir);
    // ROS_INFO_STREAM("Real dist leave = " << dist);
    if (dist > BUFFER_LEAVE)
        return true;
    return false;
}

Point VisBug21::search_intersection_point(Point A, Point B, Point C, Point D) {
    double n;
    if (B.y - A.y != 0) {
        double q = (B.x - A.x) / (A.y - B.y);
        double sn = (C.x - D.x) + (C.y - D.y);
        float fn = (C.x - A.x) + (C.y - A.y);
        n = fn / sn;
    } else {
        if (!(C.y - D.y)) return Point();
        n = (C.y - A.y) / (C.y - D.y);
    }
    Point result;
    result.x = C.x + (D.x - C.x) * n;
    result.y = C.y + (D.y - C.y) * n;
    // ROS_INFO_STREAM("Hypo intersection = " << result);
    // Проверка, что точка пересечения находится в пределах отрезков
    if ((result.x >= std::min(A.x, B.x) && result.x <= std::max(A.x, B.x)) &&
        (result.y >= std::min(A.y, B.y) && result.y <= std::max(A.y, B.y)) &&
        (result.x >= std::min(C.x, D.x) && result.x <= std::max(C.x, D.x)) &&
        (result.y >= std::min(C.y, D.y) && result.y <= std::max(C.y, D.y))) {
        ROS_INFO_STREAM("IN SEGMENT");
        return result;
    } else {
        ROS_INFO_STREAM("NOT in segment");
        // Точка пересечения находится вне пределов отрезков
        return Point(); // Возвращаем пустую точку или любой другой способ обозначения отсутствия пересечения
    }
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
    if (calc_dist_points(H, prev_H) < BUFFER_CHECK_REACHABILITY && H.x * H.y * prev_H.x * prev_H.y != 0) {
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
    else
        ROS_INFO_STREAM("Main Step 2");
}