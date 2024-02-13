// Direction for following the wall
#define DIR_IS_LEFT true
// Frequency of the main loop
#define RATE_FREQUENCY 15
// Accuracy for determening the achievement of target
#define ACCURACY_TARGET 0.2
// Minimum distance to the obstacle for determening leave point
#define DISTANCE_TO_OBSTACLE 0.25
// Minimum distance in front of the robot for determening leave point
#define FREE_SPACE_LEAVE 0.1
// Buffer just in case, to avoid boundary case in testng target reachability
#define BUFFER_CHECK_REACHABILITY 0.5
// Header, containing base class and common methods for any Bug algorithm
#include "bug.h"

// The main class for the algorithm
class VisBug21 final : public BugAlg {
public:
    VisBug21() = default;
    virtual void change_state(int input_state) override;
    virtual void main_logic() override;
    void check_leave_point();
    void check_front_wall();
    void check_reachability();
private:
    // Variable that stores path passed from the last hit point
    double last_hit_path;
    // Variable for storing last hit point
    geometry_msgs::Point hit_point;
    // Names of states for some logging messages
    const std::array<std::string, 2> MAIN_STATE_NAMES = {
            "Main Step 1: Moving towards Ti",
            "Main Step 2: Moving along obstacle boundary"};
    const std::array<std::string, 4> COMPUTE_STATE_NAMES = {
        "Compute Step 1: Target is visible",
        "Compute Step 2: Processing candidates for Ti along the M-line, defining hit points",
        "Compute Step 3: Processing candidates for Ti along the obstacle boundaries, defining leave points",
        "Compute Step 4: A special case - testing noncontiguous points of M-line as candidates for Ti"
    }
};