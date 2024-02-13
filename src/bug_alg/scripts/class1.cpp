#include "class1.h"

int main(int argc, char **argv) {
    // Initializing the Class1 algorithm with command line args
    ros::init(argc, argv, "class1");
    // Creating an instance of Class1
    Class1 class1;
    // Calling the main_logic() method to start the algorithm
    class1.main_logic();
    return EXIT_SUCCESS;
}

void Class1::check_front_wall() {
    // Checking the nearest front distance to identify an obstacle
    if (regions["front"] > 0 && regions["front"] < DISTANCE_TO_OBSTACLE) {
        ROS_INFO_STREAM("An obstacle was encountered in front.");
        // Increasing the obstacle counter
        obstacle_count++;
        // Current position is the hit point
        hit_point = cur_pos;
        // Path from the last hit point is now equal to zero
        last_hit_path = 0;
        // Activating the wall following behaviour (state 2 of algorithm)
        change_state(2);
    }
}

void Class1::check_reachability() {
    // Adding distance traversed from the last step
    last_hit_path += prev_path_delta;
    // If some distance has already been traversed + robot is in previous hit point again
    if(last_hit_path > (ACCURACY_TARGET + BUFFER_CHECK_REACHABILITY) &&
       calc_dist_points(hit_point, cur_pos) < ACCURACY_TARGET) {
        // Finishing the algorithm execution
        stop_robot();
        // Creating a report about target unreachability
        create_report("Point can not be reached.");
        // Killing the system and processes
        kill_system();
    }
}

void Class1::change_state(int input_state) {
    // Changing the state variable
    state = input_state;
    ROS_INFO_STREAM("State changed: " << STATE_NAMES[state - 1]);
}

void Class1::check_leave_point() {
    // Checking the ability to leave obstacle
    if (regions["front"] > FREE_SPACE_LEAVE &&
        check_obstacle(desired_yaw) > DISTANCE_TO_OBSTACLE &&
        calc_dist_points(cur_pos, goal_point) < calc_dist_points(hit_point, goal_point)){
        ROS_INFO_STREAM("Leave point determined.");
        // Changing the behaviour to going to the goal (state 1 of the algorithm)
        change_state(1);
    }
}

void Class1::main_logic() {
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