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
        switch (state) {
            // Going to the point && checking wall in front of the robot
            case 1:
                go_to_point();
                check_front_wall();
                break;
            // Following the wall && test target reachability && finding leave point
            case 2:
                wall_follower();
                check_reachability();
                check_leave_point();
                break;
        }
        // Updating the main loop
        ros::spinOnce();
        // Sleeping for 1/RATE_FREQUENCY seconds
        rate.sleep();
    }
}

ros::ServiceClient srv_client_go_to_point_;
ros::ServiceClient srv_client_wall_follower_;
std::string return_ = "NORMAL";
double yaw_ = 0;
double sum_yaw = 0;
const double yaw_error_allowed_ = 5 * (M_PI / 180)
geometry_msgs::Point st_position_;
geometry_msgs::Point st_point;
geometry_msgs::Point position_;
int glob_steps = 0;

geometry_msgs::Point initial_position_;
initial_position_.x = ros::param::param<double>("initial_x", 0);
initial_position_.y = ros::param::param<double>("initial_y", 0);
initial_position_.z = 0;

geometry_msgs::Point desired_position_;
desired_position_.x = ros::param::param<double>("des_pos_x", 0);
desired_position_.y = ros::param::param<double>("des_pos_y", 0);
desired_position_.z = 0;

geometry_msgs::Point local_target_point;
local_target_point.x = initial_position_.x
local_target_point.y = initial_position_.y
local_target_point.z = initial_position_.z

geometry_msgs::Point desired_position_local;
desired_position_local.x = ros::param::param<double>("des_pos_x", 0);
desired_position_local.y = ros::param::param<double>("des_pos_y", 0);
desired_position_local.z = 0;

geometry_msgs::Point point_H;
point_H.x = 0;
point_H.y = 0;
point_H.z = 0;


geometry_msgs::Point point_H_check;
point_H_check.x = 0;
point_H_check.y = 0;
point_H_check.z = 0;

geometry_msgs::Point point_X;
point_X.x = 0;
point_X.y = 0;
point_X.z = 0;

geometry_msgs::Point point_Q;
point_Q.x = 0;
point_Q.y = 0;
point_Q.z = 0;

geometry_msgs::Point point_P;
point_P.x = 0;
point_P.y = 0;
point_P.z = 0;

const double radius = 10;
std::map<std::string, float> regions_;
std::vector<std::string> state_desc_ = {"Go to point", "wall following", "checking leave point", "start"};
int state_ = 0;
int count_state_time_ = 0; // seconds the robot is in a state
int count_loop_ = 0;
int count_point = 0;

void clbk_odom(const nav_msgs::Odometry::ConstPtr& msg) {
    position_ = msg->pose.pose.position;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_ = yaw;
}


double angle_increment;
double angle_min;
std::vector<float> left_range_;
std::vector<float> fleft_range_;
std::vector<float> front_range_1;
std::vector<float> front_range_2;
std::vector<float> fright_range_;
std::vector<float> right_range_;

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg) {
    angle_increment = msg->angle_increment;
    angle_min = msg->angle_min;
    
    // Copying the ranges into the respective vector variables
    left_range_ = std::vector<float>(msg->ranges.begin() + 54, msg->ranges.begin() + 89);
    fleft_range_ = std::vector<float>(msg->ranges.begin() + 18, msg->ranges.begin() + 53);
    front_range_1 = std::vector<float>(msg->ranges.begin(), msg->ranges.begin() + 10);
    front_range_2 = std::vector<float>(msg->ranges.end() - 10, msg->ranges.end());
    fright_range_ = std::vector<float>(msg->ranges.begin() + 306, msg->ranges.begin() + 341);
    right_range_ = std::vector<float>(msg->ranges.begin() + 270, msg->ranges.begin() + 305);
    
    // Calculating the minimum distance for each region
    regions_["left"] = *std::min_element(left_range_.begin(), left_range_.end());
    regions_["fleft"] = *std::min_element(fleft_range_.begin(), fleft_range_.end());
    regions_["front"] = *std::min_element(front_range_1.begin(), front_range_1.end());
    regions_["fright"] = *std::min_element(fright_range_.begin(), fright_range_.end());
    regions_["right"] = *std::min_element(right_range_.begin(), right_range_.end());

    // Clamping the values to the defined radius
    for (auto& region : regions_) {
        region.second = std::min(region.second, radius);
    }
}


void change_state(int state) {
    count_state_time_ = 0;
    state_ = state;
    
    // Informing user that the state has changed
    ROS_INFO("state changed: %s", state_desc_[state].c_str());
    
    // Different states turn on and off different services (other scripts)
    std_srvs::SetBool srv;
    srv.request.data = (state_ == 0);
    srv_client_go_to_point_.call(srv);

    srv.request.data = (state_ == 1);
    srv_client_wall_follower_.call(srv);

    if (state_ == 2 || state_ == 3) {
        srv.request.data = false;
        srv_client_go_to_point_.call(srv);
        srv_client_wall_follower_.call(srv);
    }
}


double distance_to_line(geometry_msgs::Point p0) {
   geometry_msgs::Point p1;
   geometry_msgs::Point p2;
   double up_eq;
   double lo_eq;
   double distance;

   p1 = st_position_;
   p2 = desired_position_;

   up_eq = std::fabs(
       (p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x)
   );
   lo_eq = std::sqrt(std::pow(p2.y - p1.y, 2) + std::pow(p2.x - p1.x, 2));

   distance = up_eq / lo_eq;

   return distance;
}


double calc_dist_points(geometry_msgs::Point point1, geometry_msgs::Point point2) {
   double dist;
   dist = std::sqrt(std::pow(point1.y - point2.y, 2) + std::pow(point1.x - point2.x, 2));
   return dist;
}


double normalize_angle(double angle) {
   if (std::fabs(angle) > M_PI) {
       angle = angle - (2 * M_PI * angle) / (std::fabs(angle));
   }

   return angle;
}


std::string list_to_string(const std::vector<geometry_msgs::Point>& list_points) {
    std::ostringstream str1;
    str1 << "[";
    for (size_t i = 0; i < list_points.size(); ++i) {
        if (i != 0) {
            str1 << "; ";
        }
        str1 << list_points[i].x << " " << list_points[i].y;
    }
    str1 << "]";
    return str1.str();
}


size_t process_memory() {
    using std::ios_base;
    using std::ifstream;
    using std::string;

    int pid = getpid();
    ifstream stat_stream("/proc/" + std::to_string(pid) + "/statm", ios_base::in);

    size_t result = 0;
    size_t resident = 0;
    if (stat_stream) {
        stat_stream >> result >> resident; // Ignore 'result' and parse 'resident' field.
    }

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
    size_t rss = resident * page_size_kb;
    return rss;
}


std::optional<std::pair<double, double>> intersect(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2,
                                                   const geometry_msgs::Point& p3, const geometry_msgs::Point& p4) {
    double x1 = p1.x, y1 = p1.y;
    double x2 = p2.x, y2 = p2.y;
    double x3 = p3.x, y3 = p3.y;
    double x4 = p4.x, y4 = p4.y;

    double denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    if (denom == 0) {
        return std::nullopt; // Lines are parallel
    }
    double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
    double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;
    if (ua < 0 || ua > 1 || ub < 0 || ub > 1) {
        return std::nullopt; // Intersection not within the segments
    }
    return std::make_pair(x1 + ua * (x2 - x1), y1 + ua * (y2 - y1));
}


int cmp(int a, int b) {
    return (a > b) - (a < b);
}


bool is_between(const geometry_msgs::Point& a, const geometry_msgs::Point& b, const geometry_msgs::Point& c) {
    return ((b.x - a.x) * (c.y - a.y) == (c.x - a.x) * (b.y - a.y) && 
            std::fabs(cmp(a.x, c.x) + cmp(b.x, c.x)) <= 1 &&
            std::fabs(cmp(a.y, c.y) + cmp(b.y, c.y)) <= 1);
}


bool isTargetReached(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return std::round(a.x) == std::round(b.x) && std::round(a.y) == std::round(b.y);
}


void step_4() {
    std::string return_ = "FAILURE"; // Assuming FAILURE is a possible return state
    if (is_between(initial_position_, desired_position_, local_target_point)) {
        point_Q = local_target_point;
        step_2();
    } else {
        point_Q = point_X;
        return_ = "SUCCESS";
    }
}


bool isVisible() {
    geometry_msgs::Point local_x;
    local_x.x = 0.0;

    if (desired_position_.x >= 0) {
        local_x.x = position_.x + 10;
    } else {
        local_x.x = position_.x - 10;
    }

    if (regions_["front"] >= radius && local_x.x >= desired_position_.x) {
        return true;
    } else if (position_.x + regions_["front"] >= desired_position_.x && local_x.x >= desired_position_.x) {
        return true;
    } else {
        return false;
    }
}


void step_3() {
    std::string reg = "";
    float distance_front = 0.0;

    if (regions_["front"] < radius) {
        distance_front = regions_["front"];
    } else if (regions_["fleft"] < radius && distance_front < regions_["fleft"]) {
        distance_front = regions_["fleft"];
        reg = "FLEFT";
    } else if (regions_["left"] < radius && distance_front < regions_["left"]) {
        distance_front = regions_["left"];
        reg = "LEFT";
    } else if (regions_["fright"] < radius && distance_front < regions_["fright"]) {
        distance_front = regions_["fright"];
        reg = "FRIGHT";
    } else if (regions_["right"] < radius && distance_front < regions_["right"]) {
        distance_front = regions_["right"];
        reg = "RIGHT";
    } else {
        distance_front = 0.0;
    }

    double angle_Q = angle_increment + (3 * angle_min);
    point_Q.x = distance_front * cos(angle_Q);
    point_Q.y = distance_front * sin(angle_Q);

    auto intersection = intersect(initial_position_, desired_position_, local_target_point, point_Q);
    if (intersection) {
        point_P.x = intersection->first;
        point_P.y = intersection->second;
        if (calc_dist_points(point_P, desired_position_) < calc_dist_points(point_H, desired_position_)) {
            point_X = point_P;
            auto new_intersection = intersect(point_P, desired_position_, local_target_point, point_Q);
            if (new_intersection && new_intersection->first != intersection->first && new_intersection->second != intersection->second) {
                point_H_check = point_H;
                local_target_point = point_P;
            }
        }
    }
    if (point_H_check == point_H && is_between(local_target_point, point_Q, point_H)) {
        return_ = "BAD";
    } else {
        local_target_point = point_Q;
        step_4();
        // Call step_4 or handle the next step based on your program's logic.
    }
}


void step_2() {
    float distance_along_Mline_from_Ti = regions_["front"];
    if (distance_along_Mline_from_Ti < radius) {
        local_target_point.x = point_H.x = point_X.x = desired_position_.x;
        local_target_point.y = point_H.y = point_X.y = position_.y + distance_along_Mline_from_Ti;
        step_3();
    } else {
        local_target_point.x = desired_position_.x;
        local_target_point.y = position_.y + radius;
        step_4();
    }
}


void computeTi_21() {
    glob_steps = 0;

    // Calculate the yaw to the destination point
    double desired_yaw = std::atan2(desired_position_.y - position_.y,
                                    desired_position_.x - position_.x);
    double err_yaw = desired_yaw - yaw_;

    if (isVisible()) {
        local_target_point.x = desired_position_.x;
        local_target_point.y = desired_position_.y;
        ROS_INFO("target is visible");
        return_ = "SUCCESS";
        change_state(0);
    } else if (state_ == 1) {
        ROS_INFO_STREAM(local_target_point);
        step_3();
    } else {
        ROS_INFO_STREAM(local_target_point);
        step_2();
    }

    if (return_ == "BAD") {
        std::string log = "point cannot be reached";
        ROS_INFO_STREAM(log);
        return;
    } else {
        local_target_point.x = std::round(local_target_point.x * 10.0) / 10.0;
        local_target_point.y = std::round(local_target_point.y * 10.0) / 10.0;
        // Assuming desired_position_local is a global variable
        desired_position_local = local_target_point;
    }

    if (glob_steps == 10) {
        ros::param::set("des_pos_x", desired_position_.x);
        ros::param::set("des_pos_y", desired_position_.y);
        glob_steps = 0;
    }

    change_state(0);
}


bool reset_world() {
    std_srvs::Empty srv;
    return ros::service::call("/gazebo/reset_world", srv);
}


int main(int argc, char **argv) {
    pid_t pid = getpid();
    ros::init(argc, argv, "visbug21");
    ros::NodeHandle nh;

    // Publisher to change velocities
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Initialize subscribers to laser and odometry
    ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, clbk_laser);
    ros::Subscriber sub_odom = nh.subscribe("/odom", 1000, clbk_odom);

    ros::service::waitForService("/go_to_point_switch");
    ros::service::waitForService("/wall_follower_switch");
    ros::service::waitForService("/gazebo/set_model_state");

    // Initialize service clients for other scripts involved
    srv_client_go_to_point_ = nh.serviceClient<std_srvs::SetBool>("/go_to_point_switch");
    srv_client_wall_follower_ = nh.serviceClient<std_srvs::SetBool>("/wall_follower_switch");
    srv_client_set_model_state = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient reset_world = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

    // Set robot position
    gazebo_msgs::SetModelState model_state;
    model_state.model_name = "turtlebot3_burger";
    auto timer_hp = std::chrono::system_clock::now();
    int OBSTACLES_COUNT = 0;
    std::vector<geometry_msgs::Point> POINTS;
    size_t mem_before = process_memory();
    change_state(3);

    int stuck_error = 0;
    auto stuck_pos = position_;
    st_position_ = position_;

    // Calculate the yaw to the destination point
    double desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
    double err_yaw = desired_yaw - yaw_;

    // Point the robot to the destination point
    geometry_msgs::Twist twist_msg;
    while (fabs(err_yaw) > M_PI / 90) {
        desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
        err_yaw = desired_yaw - yaw_;
        twist_msg.angular.z = err_yaw > 0 ? 0.7 : -0.7;
        pub.publish(twist_msg);
        twist_msg.angular.z = 0;
        pub.publish(twist_msg);
    }

    while (regions_.empty()) ros::spinOnce();
    computeTi_21();
    double yaw_before = yaw_;

    ros::Rate rate(20);
    while (ros::ok()) {
        if (regions_.empty()) {
            ros::spinOnce();
            continue;
        }
        glob_steps += 1;
        double distance_position_to_line = distance_to_line(position_);

        float diff = std::fabs(yaw_ - yaw_before);
        if (diff > 1) {
            sum_yaw += std::fabs(yaw_ + yaw_before);
        } else {
            sum_yaw += std::fabs(yaw_before - yaw_);
        }
        yaw_before = yaw_;

        if (std::sqrt(std::pow(desired_position_.y - position_.y, 2) + std::pow(desired_position_.x - position_.x, 2)) < 0.15) {
            geometry_msgs::Twist twist_msg;
            twist_msg.angular.z = 0;
            twist_msg.linear.x = 0;
            pub.publish(twist_msg);
            std::string log = "point reached";
            ROS_INFO("%s", log.c_str());
            POINTS.push_back(position_);
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - timer_hp;
            std::ofstream results_file;
            results_file.open("results.txt");

            // Write results to the file
            results_file << "ELAPSED TIME: " << elapsed_seconds.count() << "\n";
            results_file << "OBSTACLES COUNT: " << OBSTACLES_COUNT << "\n";
            // Assume list_to_string is a function that converts POINTS to a string
            results_file << "POINTS: " << list_to_string(POINTS) << "\n";
            // Assume process_memory returns current memory usage
            results_file << "MEMORY USAGE:  " << process_memory() - mem_before << "\n";
            results_file << "COMPLEXITY: HARD" << "\n";
            // Calculation time is not shown in the Python snippet, so it's omitted
            results_file << "TOTAL TURN: " << sum_yaw << "\n";
            
            // Close the file
            results_file.close();

            // System calls to kill nodes and reset the world
            system("rosnode kill /go_to_point");
            system("rosnode kill /wall_follower");
            reset_world();
            system(("kill " + std::to_string(pid)).c_str()); // Assuming 'pid' is the process ID
        } else if (state_ == 0) {
            // Check if there is an obstacle forward
            if (regions_["front"] > 0 && regions_["front"] < 0.25) {
                st_point = position_;
                OBSTACLES_COUNT += 1;
                stuck_error = 0;
                change_state(1);
            }
            if (count_state_time_ > 20 && count_state_time_ % 20 == 0) {
                stuck_pos = position_;
                if (calc_dist_points(stuck_pos, stuck_prev_pos) < 0.1) {
                    stuck_error += 1;
                }
                if (stuck_error > 5) {
                    std::string log = "Stuck error";
                    ROS_INFO("%s", log.c_str());
                    ros::Duration RESULT_TIME = ros::Time::now() - timer_hp;
                    std::ofstream results_file;
                    results_file.open("results.txt");
                    results_file << "ELAPSED TIME: " << RESULT_TIME << "\n";
                    results_file << "OBSTACLES COUNT: " << OBSTACLES_COUNT << "\n";
                    results_file << "POINTS: " << list_to_string(POINTS) << "\n";
                    size_t mem_after = process_memory();
                    results_file << "MEMORY USAGE:  " << (mem_after - mem_before) << "\n";
                    results_file << "COMPLEXITY: MEDIUM\n";
                    results_file << "CALCULATION TIME: -\n";
                    results_file << "TOTAL TURN: " << sum_yaw << "\n";
                    results_file.close();
                    system("rosnode kill /go_to_point");
                    system("rosnode kill /wall_follower");
                    system("rosnode kill /wall_follower_left");
                    reset_world();
                    system(("kill " + std::to_string(pid)).c_str());
                }
                stuck_prev_pos = stuck_pos;
            }
        } else if (state_ == 1) {
            stuck_pos = position_;
            if (count_state_time_ > 20 &&
                distance_position_to_line < 0.1 &&
                calc_dist_points(position_, desired_position_) < calc_dist_points(st_point, desired_position_)) {
                computeTi_21();
                change_state(2);
            }
            if (count_state_time_ > 20 && count_state_time_ % 50 == 0) {
                stuck_pos = position_;
                if (calc_dist_points(stuck_pos, stuck_prev_pos) < 0.1) {
                    stuck_error += 1;
                    ROS_INFO("stuck error: %d", stuck_error);
                }
                if (stuck_error > 5) {
                    std::string log = "Stuck error";
                    ROS_INFO("%s", log.c_str());
                    ros::Duration RESULT_TIME = ros::Time::now() - timer_hp;
                    std::ofstream results_file;
                    results_file.open("results.txt");
                    results_file << "ELAPSED TIME: " << RESULT_TIME << "\n";
                    results_file << "OBSTACLES COUNT: " << OBSTACLES_COUNT << "\n";
                    results_file << "POINTS: " << list_to_string(POINTS) << "\n";
                    size_t mem_after = process_memory();
                    results_file << "MEMORY USAGE: " << (mem_after - mem_before) << "\n";
                    results_file << "COMPLEXITY: MEDIUM\n";
                    results_file << "CALCULATION TIME: -\n";
                    results_file << "TOTAL TURN: " << sum_yaw << "\n";
                    results_file.close();
                    system("rosnode kill /go_to_point");
                    system("rosnode kill /wall_follower");
                    system("rosnode kill /wall_follower_left");
                    reset_world();
                    system(("kill " + std::to_string(pid)).c_str());
                }
                stuck_prev_pos = stuck_pos;
            }

            if ((count_state_time_ > 20 && calc_dist_points(st_point, position_) < 0.1) || return_ == "BAD") {
                std::string log = "point cannot be reached";
                POINTS.push_back(position_);
                ROS_INFO("%s", log.c_str());
                ros::Duration RESULT_TIME = ros::Time::now() - timer_hp;
                std::ofstream results_file;
                results_file.open("results.txt");
                results_file << "ELAPSED TIME: " << RESULT_TIME << "\n";
                results_file << "OBSTACLES COUNT: " << OBSTACLES_COUNT << "\n";
                results_file << "POINTS: " << list_to_string(POINTS) << "\n";
                size_t mem_after = process_memory();
                results_file << "MEMORY USAGE: " << (mem_after - mem_before) << "\n";
                results_file << "COMPLEXITY: EASY\n";
                results_file << "CALCULATION TIME: -\n";
                results_file << "TOTAL TURN: " << sum_yaw << "\n";
                results_file.close();
                system("rosnode kill /go_to_point");
                system("rosnode kill /wall_follower");
                reset_world();
                system(("kill " + std::to_string(pid)).c_str());
            }
        } else if (state_ == 2) {
            desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
            err_yaw = desired_yaw - yaw_;
            geometry_msgs::Twist twist_msg;

            while (std::fabs(err_yaw) > M_PI / 90) {
                desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
                err_yaw = desired_yaw - yaw_;
                twist_msg.angular.z = (err_yaw > 0) ? 0.7 : -0.7;
                pub.publish(twist_msg);
                twist_msg.angular.z = 0;
                pub.publish(twist_msg);
                double diff = std::fabs(yaw_ - yaw_before);

                if (diff > 1) {
                    sum_yaw += std::fabs(yaw_ + yaw_before);
                } else {
                    sum_yaw += std::fabs(yaw_before - yaw_);
                }
                yaw_before = yaw_;
            }

            if (regions_["front"] > 0.1 && regions_["front"] < 0.25) {
                std::string log = "not the leave point, continuing circumnavigating";
                ROS_INFO("%s", log.c_str());
                computeTi_21();
                change_state(1);
            } else {
                computeTi_21();
            }
        }

        count_loop_ += 1;
        if (count_loop_ == 20) {
            count_state_time_ += 1;
            count_loop_ = 0;
        }
        
        count_point += 1;
        if (count_point == 25) {
            count_point = 0;
            POINTS.push_back(position_);
        }
        rate.sleep();
    }
}