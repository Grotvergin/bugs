// Direction for following the wall
#define DIR_IS_LEFT true
// Frequency of the main loop
#define RATE_FREQUENCY 15
// Accuracy for determening the achievement of target
#define ACCURACY_TARGET 0.2
// Accuracy for reaching Ti
#define ACCURACY_CUR_POS_IS_Ti 0.1
// Minimum distance to the obstacle for determening leave point
#define DISTANCE_TO_OBSTACLE 0.25
// Minimum distance in front of the robot for determening leave point
#define FREE_SPACE_LEAVE 0.1
// Buffer just in case, to avoid boundary case in testng target reachability
#define BUFFER_CHECK_REACHABILITY 0.5
#define VISION_RADIUS 1
// Header, containing base class and common methods for any Bug algorithm
#include "bug.h"

// The main class for the algorithm
class VisBug21 final : public BugAlg {
public:
    VisBug21() = default;
    virtual void main_logic() override;
    void change_state_alg(int input_state);
    void change_state_procedure(int input_state);
    bool cur_pos_is_Ti();
    void computeTi21();
    bool goal_is_visible();
    void check_leave_point();
    void check_front_wall();
    void check_reachability();
    bool point_is_on_boundary(geometry_msgs::Point point);
    geometry_msgs::Point search_endpoint_segment_Mline();
    geometry_msgs::Point search_endpoint_segment_boundary();
    bool boundary_crosses_Mline();
    geometry_msgs::Point search_boundary_Mline_intersection_point();
    bool segment_crosses_obstacle(geometry_msgs::Point A, geometry_msgs::Point B);
    bool point_is_on_Mline(geometry_msgs::Point point);
    bool Mline_is_seen();
    geometry_msgs::Point search_closest_to_goal_Mline_point();
    bool is_in_main_semiplane();

private:
    // Variable for intermediate temporary target points
    geometry_msgs::Point Ti_pos, Q_pos, H_pos, X_pos, P_pos, L_pos, S_apostrophe_point;
    // Variable for storing the state of the procedure
    int procedure_state;
    // Names of states for some logging messages
    const std::array<std::string, 2> MAIN_STATE_NAMES = {
            "Main Step 1: Moving towards Ti",
            "Main Step 2: Moving along obstacle boundary"};
    const std::array<std::string, 4> PROCEDURE_STATE_NAMES = {
        "Compute Step 1: Target is visible",
        "Compute Step 2: Processing candidates for Ti along the M-line, defining hit points",
        "Compute Step 3: Processing candidates for Ti along the obstacle boundaries, defining leave points",
        "Compute Step 4: A special case - testing noncontiguous points of M-line as candidates for Ti"
    };
};