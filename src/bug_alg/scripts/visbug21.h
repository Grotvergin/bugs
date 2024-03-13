// Direction for following the wall
#define DIR_IS_LEFT true
// Frequency of the main loop
#define RATE_FREQUENCY 1
// Accuracy for determening the achievement of target
#define ACCURACY_TARGET 0.2
// Accuracy for reaching Ti
#define ACCURACY_CUR_POS_IS_Ti 0.01
// Minimum distance to the obstacle for determening leave point
#define DISTANCE_TO_OBSTACLE 0.25
// Minimum distance in front of the robot for determening leave point
#define FREE_SPACE_LEAVE 0.1
// Buffer just in case, to avoid boundary case in testng target reachability
#define BUFFER_CHECK_REACHABILITY 0.05
// Radius of vision for the robot
#define VISION_RADIUS 0.5f
// Accuracy for determening whether the point lies on Mline
#define ACCURACY_MLINE 0.2
// Step for moving along Mline to search appropriate Mline point
#define STEP_MLINE 0.1
#define ACCURACY_LINES 0.01f
#define DELTA_OBSTACLE_DECISION 0.05
// Header, containing base class and common methods for any Bug algorithm
#include "bug.h"

// The main class for the algorithm
class VisBug21 final : public BugAlg {
public:
    VisBug21() = default;
    virtual void main_logic() override;
    void change_state(int input_state);
    void computeTi21();
    char procedure_step_1();
    char procedure_step_2();
    char procedure_step_3();
    char procedure_step_4();
    bool cur_pos_is_Ti();
    bool goal_is_visible();
    bool is_in_main_semiplane();
    void check_reachability();
    virtual void clbk_laser(const sensor_msgs::LaserScan::ConstPtr &msg);
    bool point_is_on_Mline(geometry_msgs::Point point);
    geometry_msgs::Point search_endpoint_segment_Mline();
    geometry_msgs::Point math_search_endpoint_Mline();
    void move_along_Mline();
    bool segment_crosses_Mline(geometry_msgs::Point A, geometry_msgs::Point B);
    bool is_between(double x, double b1, double b2);
    double search_angle_endpoint_segment_boundary(const sensor_msgs::LaserScan::ConstPtr &msg);
    geometry_msgs::Point search_intersection_point(geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C, geometry_msgs::Point D);
    bool segment_not_crosses_obstacle(geometry_msgs::Point A, geometry_msgs::Point B);
    geometry_msgs::Point search_endpoint_segment_boundary();
    bool point_is_on_boundary(geometry_msgs::Point point);

private:
    double yaw_goal, degree_goal, yaw_endpoint_Mline, degree_endpoint_Mline, angle_to_boundary, angle_for_decision;
    // Variable for intermediate temporary target points
    geometry_msgs::Point Ti_pos, Q_pos, H_pos, X_pos, P_pos, L_pos, S_apostrophe_point, prev_H_pos, potential_Mline_point;
};