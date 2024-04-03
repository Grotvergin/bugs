// Direction for following the wall
#define DIR_IS_LEFT true
// Frequency of the main loop
#define RATE_FREQUENCY 2
// Accuracy for determening the achievement of target
#define ACCURACY_TARGET 0.2
// Accuracy for reaching Ti
#define ACCURACY_CUR_POS_IS_Ti 0.5
// Buffer just in case, to avoid boundary case in testng target reachability
#define BUFFER_CHECK_REACHABILITY 0.03
// Radius of vision for the robot (CHANGE BASE DIST TOO!!!!)
#define VISION_RADIUS 0.8f
// Accuracy for determening whether the point lies on Mline
#define ACCURACY_MLINE 0.1
// Step for moving along Mline to search appropriate Mline point
#define SENSIVITY_MOVE_MLINE 0.01
#define ACCURACY_LINES 0.01f
#define DELTA_OBSTACLE_DECISION 0.3
#define BUFFER_LEAVE 0.2
#define RANGE_SEARCH_DEGREES 30
#define SECTOR_OBSTACLE_DECISION 1
#define DISTANCE_TO_OBSTACLE 0.3
// Header, containing base class and common methods for any Bug algorithm
#include "bug.h"
using Point = geometry_msgs::Point;

// The main class for the algorithm
class VisBug21 final : public BugAlg {
public:
    // High-level functions
    VisBug21() = default;
    virtual void main_logic() override;
    void change_state(int input_state);
    virtual void clbk_laser(const sensor_msgs::LaserScan::ConstPtr &msg);
    void refresh();
    void show_points();
    // Procedure Compute Ti-21 and its steps
    void computeTi21();
    char procedure_step_1();
    char procedure_step_2();
    char procedure_step_3();
    char procedure_step_4();
    // Lower-level functions 
    void move_Ti(Point hypo_Ti);
    bool cur_pos_is_Ti();
    bool goal_is_visible();
    bool is_in_main_semiplane();
    bool point_is_on_boundary(Point point);
    bool point_is_on_Mline(Point point);
    bool is_between(double x, double b1, double b2);
    bool enough_space_to_leave();
    bool near_obstacle();
    void check_reachability();
    // Returning points functions
    Point search_endpoint_segment_Mline();
    Point math_search_endpoint_Mline();
    Point move_along_Mline(Point point);
    Point search_intersection_point(Point A, Point B, Point C, Point D);
    Point search_endpoint_segment_boundary();
    // Working with different coordinate systems
    int radian2degree(double radian);
    double degree2radian(int degree);
    int adapt_degree(double interested_radian);
    double get_distance_by_course(double interested_radian);
private:
    // Variables for intermediate temporary target points
    Point Ti, Q, H, X, P, L, S, prev_H;
    bool leaving_obstacle = false;
};