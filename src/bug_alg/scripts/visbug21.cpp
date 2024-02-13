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
        switch(alg_state) {
            // Moving towards Ti && computing procedure && checking cur_pos = Ti
            case 1:
                go_to_point();
                break;
            // Moving along obstacle boundary && computing procedure && checking cur_pos != Ti
            case 2:
                break;


        }
        // Updating the main loop
        ros::spinOnce();
        // Sleeping for 1/RATE_FREQUENCY seconds
        rate.sleep();
    }
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