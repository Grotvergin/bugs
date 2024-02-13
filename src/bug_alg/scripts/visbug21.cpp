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
    change_state_alg(1);
    while (nh.ok()) {
        // Recording the change of yaw/position
        monitor_indicators();
        // Performing check of reaching the target
        check_if_goal_is_reached();
        // Computing procedure ComputeTi-21
        computeTi21();
        switch (alg_state) {
            case 1:
                // Moving towards Ti 
                go_to_point(Ti_pos);
                // Checking cur_pos = Ti
                if (cur_pos_is_Ti())
                    change_state_alg(2);
                break;
            case 2:
                // Moving along obstacle boundary
                wall_follower();
                // Checking cur_pos != Ti
                if (!cur_pos_is_Ti())
                    change_state_alg(1);
                break;
        }
        // Updating the main loop
        ros::spinOnce();
        // Sleeping for 1/RATE_FREQUENCY seconds
        rate.sleep();
    }
}

void VisBug21::computeTi21() {
    switch (procedure_state) {
        // The last stage when target is visible
        case 1:
            // Checking the target visibility
            if (goal_is_visible())
                Ti_pos = goal_point;
            // Checking if Ti is on an obstacle boundary
            else if (Ti_is_on_boundary())
                change_state_procedure(3);
            // In all other cases going to step 2
            else
                change_state_procedure(2);
            break;
        // Candidates for Ti along the M-line are processed && hit points are defined
        case 2:
            
            break;
        // Candidates for Ti along obstacle boundaries are processed && leave points are defined
        case 3:
            break;
        // Candidates for Ti - points of M-line noncontiguous to previous sets of points - are processed
        case 4:
            break;
    }
}

bool Ti_is_on_boundary() {

}

bool VisBug21::cur_pos_is_Ti() {
    if (calc_dist_points(Ti_pos, cur_pos) < ACCURACY_CUR_POS_IS_Ti)
        return true;
    return false;
}

void VisBug21::check_reachability() {

}

void VisBug21::change_state_alg(int input_state) {
    alg_state = input_state;
    ROS_INFO_STREAM("Algorithm state changed: " << MAIN_STATE_NAMES[alg_state]);
}

void VisBug21::change_state_procedure(int input_state) {
    procedure_state = input_state;
    ROS_INFO_STREAM("Algorithm state changed: " << PROCEDURE_STATE_NAMES[procedure_state]);
}