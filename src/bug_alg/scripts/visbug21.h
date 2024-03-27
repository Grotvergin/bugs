// Direction for following the wall
#define DIR_IS_LEFT true
// Frequency of the main loop
#define RATE_FREQUENCY 1
// Accuracy for determening the achievement of target
#define ACCURACY_TARGET 0.2
// Accuracy for reaching Ti
#define ACCURACY_CUR_POS_IS_Ti 0.01
// Buffer just in case, to avoid boundary case in testng target reachability
#define BUFFER_CHECK_REACHABILITY 0.05
// Radius of vision for the robot
#define VISION_RADIUS 0.5f
// Accuracy for determening whether the point lies on Mline
#define ACCURACY_MLINE 0.2
// Step for moving along Mline to search appropriate Mline point
#define SENSIVITY_MOVE_MLINE 0.01
#define ACCURACY_LINES 0.01f
#define DELTA_OBSTACLE_DECISION 0.05
#define DELTA_SEGM_CR_OBST 0.1
#define RANGE_SEARCH_DEGREES 30
// Header, containing base class and common methods for any Bug algorithm
#include "bug.h"

// The main class for the algorithm
class VisBug21 final : public BugAlg {
public:
    // High-level functions
    VisBug21() = default;
    virtual void main_logic() override;
    void change_state(int input_state);
    virtual void clbk_laser(const sensor_msgs::LaserScan::ConstPtr &msg);
    // Procedure Compute Ti-21 and its steps
    void computeTi21();
    char procedure_step_1();
    char procedure_step_2();
    char procedure_step_3();
    char procedure_step_4();
    // Lower-level functions 
    bool cur_pos_is_Ti();
    bool goal_is_visible();
    bool is_in_main_semiplane();
    bool segment_crosses_Mline(geometry_msgs::Point A, geometry_msgs::Point B);
    bool point_is_on_boundary(geometry_msgs::Point point);
    bool point_is_on_Mline(geometry_msgs::Point point);
    bool is_between(double x, double b1, double b2);
    bool segment_crosses_obstacle();
    void check_reachability();
    // Returning points functions
    geometry_msgs::Point search_endpoint_segment_Mline();
    geometry_msgs::Point math_search_endpoint_Mline();
    geometry_msgs::Point move_along_Mline(geometry_msgs::Point point);
    geometry_msgs::Point search_intersection_point(geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C, geometry_msgs::Point D);
    geometry_msgs::Point search_endpoint_segment_boundary();
    // Working with different coordinate systems
    int radian2degree(double radian);
    int adapt_degree(double interested_radian)
    double get_distance_by_course(double interested_radian);
private:
    // Variables for intermediate temporary target points
    geometry_msgs::Point Ti_pos, Q_pos, H_pos, X_pos, P_pos, L_pos, S_apostrophe_point, prev_H_pos;
};