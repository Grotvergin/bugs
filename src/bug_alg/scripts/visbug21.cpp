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
    // Loop while nodehandler is active
    while (nh.ok()) {
        // Recording the change of yaw/position
        monitor_indicators();
        // Performing check of reaching the target
        check_if_goal_is_reached();
        // Computing procedure ComputeTi-21
        computeTi21();
        // Switching between two main states of the algorithm
        switch (state) {
            case 1:
                // Moving towards Ti 
                go_to_point(Ti);
                // Checking cur_pos = Ti or robot is near obstacle and not leaving right now
                if ((cur_pos_is_Ti() || near_obstacle()) && !leaving_obstacle)
                    // If so, changing state to second
                    change_state(2);
                break;
            case 2:
                // Moving along obstacle boundary
                wall_follower();
                // Checking cur_pos != Ti and not being near obstacle, or leaving the obstacle right now
                if (!cur_pos_is_Ti() && !near_obstacle() || leaving_obstacle)
                    change_state(1);
                break;
        }
        // Refreshing the main loop
        refresh();
    }
}


void VisBug21::computeTi21() {
    // First step execution and discovering the next step
    char next_step = procedure_step_1();
    // Iterating in the loop while next_step in not equal to zero <-> the procedure is finished
    while(next_step) {
        if (next_step == 2)
            next_step = procedure_step_2();
        else if (next_step == 3)
            next_step = procedure_step_3();
        else next_step = procedure_step_4();
    }
}


char VisBug21::procedure_step_1() {
    ROS_INFO_STREAM("----- CS 1 -----");
    // Checking the target visibility
    if (goal_is_visible()) {
        // Going straightforward to the obstacle
        Ti = goal_point;
        // Finishing the procedure immediately
        return 0;
    }
    // Checking if Ti is on an obstacle boundary
    if (point_is_on_boundary(Ti))
        // Going to the step 3 of procedure
        return 3;
    // In all other cases going to step 2 of procedure
    return 2;
}


char VisBug21::procedure_step_2() {
    ROS_INFO_STREAM("----- CS 2 -----");
    // Searching for the furthest visible point of Mline
    Q = search_endpoint_segment_Mline();
    // Assigning Q to Ti via speacial method
    move_Ti(Q);
    // Checking whether the obstacle was hit
    if (point_is_on_boundary(Q) && state == 1) {
        // Assigning points according to the algorithm
        prev_H = H;
        H = Q;
        if (IS_21)
            X = Q;
        ROS_INFO_STREAM("!HP!: x = " << H.x << " y = " << H.y);
        // Going to step 3 of procedure
        return 3;
    }
    // If the condition is not satisfied, going to step 4 of procedure
    return 4;
}


char VisBug21::procedure_step_3() {
    ROS_INFO_STREAM("----- CS 3 -----");
    // Searching for the furthest visible point of boundary
    Q = search_endpoint_segment_boundary();
    // Searching for possible intersection of SG (start_point and goal_point) and TiQ
    Point hypo_P = search_intersection_point(start_point, goal_point, Ti, Q);
    // If intersection is found or Q is on Mline, and the robot is following the wall
    if ((hypo_P.x * hypo_P.y != 0 || point_is_on_Mline(Q)) && state == 2) {
        // Finding out which point should be assigned to Ti
        if (hypo_P.x * hypo_P.y != 0)
            P = hypo_P;
        else if (point_is_on_Mline(Q))
            P = Q;
        // If the found P point is closer to goal than H point
        if (calc_dist_points(P, goal_point) < calc_dist_points(H, goal_point)) {
            if (IS_21)
                X = P;
            // If there is enough space in the desired direction
            if (enough_space_to_leave()) {
                L = P;
                // Extra variable for better behaviour near boundaries
                leaving_obstacle = true;
                Ti = goal_point;
                ROS_INFO_STREAM("!LP!: x = " << L.x << " y = " << L.y);
                return 2;
            }
        }
    }
    // Checking target reachability
    // check_reachability();
    // Moving the Ti with special method
    move_Ti(Q);
    return 4;
}


char VisBug21::procedure_step_4() {
    ROS_INFO_STREAM("----- CS 4 -----");
    // Assigning Q to Ti, if Ti is on Mline, else to X
    if (IS_21)
        Q = point_is_on_Mline(Ti) ? Ti : X;
    else
        Q = point_is_on_Mline(Ti) ? Ti : H;
    // Searching for the furthest visible point of Mline
    S = search_endpoint_segment_Mline();
    // Managing a special case
    if (calc_dist_points(S, goal_point) < calc_dist_points(Q, goal_point)) {
        if (IS_21 && is_in_main_semiplane() || !IS_21) {
            // Moving the Ti with special method
            move_Ti(S);
            // Going to the step 2 of procedure
            return 2;
        }
    }
    // Finishing the procedure
    return 0;
}


// Method which guarantees movement of Ti closer to goal_point
void VisBug21::move_Ti(Point hypo_Ti) {
    if (calc_dist_points(hypo_Ti, goal_point) < calc_dist_points(Ti, goal_point))
        Ti = hypo_Ti;
}


// Method for debugging, which shows all significant points
void VisBug21::show_points() {
    ROS_INFO_STREAM("Ti: x = " << Ti.x << " y = " << Ti.y);
    ROS_INFO_STREAM("Q: x = " << Q.x << " y = " << Q.y);
    ROS_INFO_STREAM("H: x = " << H.x << " y = " << H.y);
    ROS_INFO_STREAM("X: x = " << X.x << " y = " << X.y);
    ROS_INFO_STREAM("P: x = " << P.x << " y = " << P.y);
    ROS_INFO_STREAM("L: x = " << L.x << " y = " << L.y);
    ROS_INFO_STREAM("S: x = " << S.x << " y = " << S.y);
    ROS_INFO_STREAM("prev_H: x = " << prev_H.x << " y = " << prev_H.y);
}


void VisBug21::refresh() {
    // Variable for sleeping between iterations of ros::spinOnce()
    ros::Rate rate(RATE_FREQUENCY);
    // Updating the main loop
    ros::spinOnce();
    // Sleeping for 1/RATE_FREQUENCY seconds
    rate.sleep();
}


// Method converts radians to degrees
int VisBug21::radian2degree(double radian) {
    return round(radian * 180 / M_PI);
}


// Method converts degrees to radians
double VisBug21::degree2radian(int degree) {
    return (degree * M_PI / 180.0);
}


// Adapts direction in the base system to the corresponding angle for regions
int VisBug21::adapt_degree(double interested_radian) {
    // Calculating the angle
    int angle = radian2degree(interested_radian) - radian2degree(cur_yaw);
    // Normalising the calculated angle
    if (angle < 0) angle += 360;
    return angle;
}


// Method for getting distance by interested direction
double VisBug21::get_distance_by_course(double interested_radian) {
    // Using previously defined method for finding the regions degree
    int angle = adapt_degree(interested_radian);
    // Returning distance in the desired direction
    return regions[std::to_string(angle)];
}


// Method for checking whether the robot stands near obstacle (for activating wall following behaviour)
bool VisBug21::near_obstacle() {
    // If the wall is detected anywhere in front of the robot, returning true
    return (regions["front"] < DISTANCE_TO_OBSTACLE ||
            regions["fright"] < DISTANCE_TO_OBSTACLE ||
            regions["fleft"] < DISTANCE_TO_OBSTACLE ||
            regions["right"] < DISTANCE_TO_OBSTACLE ||
            regions["left"] < DISTANCE_TO_OBSTACLE);
}


// Method for deciding whether the given point is on boundary (works at the distance)
bool VisBug21::point_is_on_boundary(Point point) {
    // Calculating the degree, where the probable point is situated
    double dir_interest = atan2(point.y - cur_pos.y, point.x - cur_pos.x);
    // Calculating the mathematical distance between current robot position and the point
    double math_dist = calc_dist_points(cur_pos, point);
    // Checking distance by 3 directions for reliability. Danger of overflow!
    return (fabs(get_distance_by_course(dir_interest) - math_dist) < DELTA_OBSTACLE_DECISION ||
            fabs(get_distance_by_course(dir_interest + SECTOR_OBSTACLE_DECISION) - math_dist) < DELTA_OBSTACLE_DECISION ||
            fabs(get_distance_by_course(dir_interest - SECTOR_OBSTACLE_DECISION) - math_dist) < DELTA_OBSTACLE_DECISION);
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


// Method for finding the furthest point of mathematical intersection of Mline and vision radius
Point VisBug21::math_search_endpoint_Mline() {
    Point point;
    // Calculating the shift for both axis
    double dx = start_point.x - goal_point.x;
    double dy = start_point.y - goal_point.y;
    // Calculating the eqution parameters
    double A = dx * dx + dy * dy;
    double B = 2 * (dx * (goal_point.x - cur_pos.x) + dy * (goal_point.y - cur_pos.y));
    double C = pow((goal_point.x - cur_pos.x), 2) + pow((goal_point.y - cur_pos.y), 2) - pow(VISION_RADIUS, 2);
    // Calculating the determinant
    double det = B * B - 4 * A * C;
    // No intersection found
    if (A <= 0.0000001 || det < 0) {
        return point;
    }
    // One intersection found
    if (fabs(det) < 0.0000001) {
        double t = -B / (2 * A);
        point.x = goal_point.x + t * dx;
        point.y = goal_point.y + t * dy;
        return point;
    }
    // Two points of intersection found, calculting the first one
    double t = (double)((-B + sqrt(det)) / (2 * A));
    point.x = goal_point.x + t * dx;
    point.y = goal_point.y + t * dy;
    t = (double)((-B - sqrt(det)) / (2 * A));
    // Calculating the second point
    Point another;
    another.x = goal_point.x + t * dx;
    another.y = goal_point.y + t * dy;
    // Deciding which point is closer to the goal to return it
    if (calc_dist_points(point, goal_point) < calc_dist_points(another, goal_point))
        return point;
    return another;

}


// Method searches the furthest point of Mline visible for the robot
Point VisBug21::search_endpoint_segment_Mline() {
    // At first we find that point as the mathematical intersection
    Point potential_Mline_point = math_search_endpoint_Mline();
    // Then we find the direction to this point
    double dir_interest = atan2(potential_Mline_point.y - cur_pos.y, potential_Mline_point.x - cur_pos.x);
    // Loop while we can not see the point directly, without boundaries
    while(calc_dist_points(cur_pos, potential_Mline_point) > get_distance_by_course(dir_interest)) {
        // Move the potential point a little closer to the robot
        potential_Mline_point = move_along_Mline(potential_Mline_point);
        // Calculating the direction to the changed potential point
        dir_interest = atan2(potential_Mline_point.y - cur_pos.y, potential_Mline_point.x - cur_pos.x);
        // Refreshing the main loop to get new data from laser
        refresh();
    }
    // Returning the found point
    return potential_Mline_point;
}


// Method moves the point a little closer to the robot
Point VisBug21::move_along_Mline(Point point) {
    // Calculating the length of the segment
    double len = calc_dist_points(start_point, point);
    // Normalising the step
    double step = len * SENSIVITY_MOVE_MLINE;
    // Calculating the shifts for both axis
    double dx = (start_point.x - point.x) / len;
    double dy = (start_point.y - point.y) / len;
    // Moving the point itself
    point.x += dx * step;
    point.y += dy * step;
    return point;
}


// Method for searching the furthest visible point of segment boundary
Point VisBug21::search_endpoint_segment_boundary() {
    Point potential_Q_point;
    // Calculating the distance to the desired direction
    double dir_Ti = atan2(Ti.y - cur_pos.y, Ti.x - cur_pos.x);
    // Finding the degree in the robot's coordinate system
    int robot_degree = adapt_degree(dir_Ti);
    // Assigning the potential angle to the robot's degree to the boundary
    int potential_angle = radian2degree(dir_Ti);
    // Calculating the initial distance to Ti
    double dist_Q = calc_dist_points(cur_pos, Ti);
    // Different local directions need different beaviour
    if (DIR_IS_LEFT) {
        // Loop for gradual movement in the local direction
        for (int i = robot_degree; i - robot_degree < RANGE_SEARCH_DEGREES; ++i) {
            // Calculating the current distance to the desired direction
            double cur_dist = regions[std::to_string(i >= 360 ? i - 360 : i)];
            // If we are still looking at the boundary
            if (cur_dist < VISION_RADIUS) {
                dist_Q = cur_dist;
                potential_angle++;
            } else {
                // We have lost the obstacle
                break;
            }
        }
        // Normalising the angle
        if (potential_angle > 180) potential_angle -= 360;
    } else {
        // Loop for gradual movement in the local direction
        for (int i = robot_degree; robot_degree - i < RANGE_SEARCH_DEGREES; --i) {
            // Calculating the current distance to the desired direction
            double cur_dist = regions[std::to_string(i < 0 ? i + 360 : i)];
            // If we are still looking at the boundary
            if (cur_dist < VISION_RADIUS) {
                dist_Q = cur_dist;
                potential_angle--;
            } else {
                // We have lost the obstacle
                break;
            }
        }
        // Normalising the angle
        if (potential_angle < -180) potential_angle += 360;
    }
    // Calculating the Q point 
    potential_Q_point.x = cur_pos.x + cos(degree2radian(potential_angle)) * dist_Q;
    potential_Q_point.y = cur_pos.y + sin(degree2radian(potential_angle)) * dist_Q; 
    return potential_Q_point;
}


// Method checks whether it is enough space in the direction of goal
bool VisBug21::enough_space_to_leave() {
    // Calculating the disrection to the goal_point
    double dir = atan2(goal_point.y - cur_pos.y, goal_point.x - cur_pos.x);
    // If there is enough distance, return true, else false
    return get_distance_by_course(dir) > BUFFER_LEAVE;
}


// Method for searching intersection between two segments
Point VisBug21::search_intersection_point(Point A, Point B, Point C, Point D) {
    double n;
    // Finding point of intersection between two straight lines
    if (B.y - A.y != 0) {
        double q = (B.x - A.x) / (A.y - B.y);
        double sn = (C.x - D.x) + (C.y - D.y);
        float fn = (C.x - A.x) + (C.y - A.y);
        n = fn / sn;
    } else {
        if (!(C.y - D.y)) return Point();
        n = (C.y - A.y) / (C.y - D.y);
    }
    // Calculating the coordinate of the found point
    Point result;
    result.x = C.x + (D.x - C.x) * n;
    result.y = C.y + (D.y - C.y) * n;
    // Deciding whether the found point is in the segment
    if ((result.x >= std::min(A.x, B.x) && result.x <= std::max(A.x, B.x)) &&
        (result.y >= std::min(A.y, B.y) && result.y <= std::max(A.y, B.y)) &&
        (result.x >= std::min(C.x, D.x) && result.x <= std::max(C.x, D.x)) &&
        (result.y >= std::min(C.y, D.y) && result.y <= std::max(C.y, D.y))) {
        return result;
    } else {
        // Out of the segment, returning empty point
        return Point();
    }
}


// Method for checking whether the given point lies on Mline
bool VisBug21::point_is_on_Mline(Point point) {
    // Calculating the components of the equation
    double A = start_point.y - goal_point.y;
    double B = goal_point.x - start_point.x;
    double C = start_point.x * goal_point.y - goal_point.x * start_point.y;
    // Finding the distance
    double distance = fabs(A * point.x + B * point.y + C) / sqrt(pow(A, 2) + pow(B, 2));
    // If the distance is lower than the accuracy, the point is situated on Mline
    return distance <= ACCURACY_MLINE;
}


// Function for checking whether the robot is in main semiplane
bool VisBug21::is_in_main_semiplane() {
    // Calculating the components of the equation
    double A = goal_point.y - start_point.y;
    double B = start_point.x - goal_point.x;
    double C = goal_point.x * start_point.y - start_point.x * goal_point.y;
    double D = A * cur_pos.x + B * cur_pos.y + C;
    // Returning the decision according to the chosen local direction
    return (D >= 0 && DIR_IS_LEFT) || (D <= 0 && !DIR_IS_LEFT);
}


// Method for checking the ability to reach the target
void VisBug21::check_reachability() {
    // If the previous hit point is encountered again, then the target is unreachable
    if (calc_dist_points(H, prev_H) < BUFFER_CHECK_REACHABILITY && H.x * H.y * prev_H.x * prev_H.y != 0) {
        stop_robot();
        create_report("Target can not be reached.");
        kill_system();
    }
}


// Method fot checking the visibility of the target
bool VisBug21::goal_is_visible() {
    // Calculating the geometrical distance to the target
    double geom_dist = calc_dist_points(cur_pos, goal_point);
    // Looking at the target and measuring free distance
    double visible_dist = get_distance_by_course(atan2(goal_point.y - cur_pos.y, goal_point.x - cur_pos.x));
    // Checking two conditions:
    // 1. Geometrical distance between goal and current is less than vision radius
    // 2. Free distance to goal is more than geometrical
    return geom_dist < VISION_RADIUS && visible_dist >= geom_dist;
}


// Method for deciding whether current position is Ti
bool VisBug21::cur_pos_is_Ti() {
    // If current position is almost Ti, returning true
    return calc_dist_points(Ti, cur_pos) < ACCURACY_CUR_POS_IS_Ti;
}


// Method for graceful changing of main states
void VisBug21::change_state(int input_state) {
    state = input_state;
    if (input_state == 1)
        ROS_INFO_STREAM("~~~~~ MS 1 ~~~~~");
    else
        ROS_INFO_STREAM("~~~~~ MS 2 ~~~~~");
}