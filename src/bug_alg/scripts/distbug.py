#! /usr/bin/env python

import time
import datetime
import os
import psutil
# import ros stuff
import rospy
import math

# import ros message
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# import ros service
from std_srvs.srv import *



#Initializing parameters
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0

#Get the innitial position coordinates
position_ = Point()
initial_position_ = Point()
initial_position_.x = rospy.get_param('initial_x')
initial_position_.y = rospy.get_param('initial_y')
initial_position_.z = 0

#Get the destination coordinates
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None

#States of robot during algorithm
state_desc_ = ['Go to point', 'wall following', 'checking leave point', 'start']
state_ = 0
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
count_point = 0
# 0 - go to point
# 1 - wall following
# 2 - checking leave point
# 3 - start position

# Array of points of previous steps
list_prev_points = []
len_prev_points = 0

# Callbacks
# Robot movement callbacks
def clbk_odom(msg):
    global position_, yaw_
    
    # Position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


# Laser callback
def clbk_laser(msg):
    global regions_
    regions_ = {
        'bbleft': min(min(msg.ranges[136:180]), 10),
        'bleft':  min(min(msg.ranges[103:135]), 10),
        'left':  min(min(msg.ranges[57:102]), 10),
        'fleft': min(min(msg.ranges[26:56]), 10),
        'front':  min(min(min(msg.ranges[0:25]), min(msg.ranges[334:359])) , 10),
        'fright':  min(min(msg.ranges[303:333]), 10),
        'right':   min(min(msg.ranges[257:302]), 10),
        'bright':  min(min(msg.ranges[225:256]), 10),
        'bbright':  min(min(msg.ranges[179:224]), 10),
    }

# State changer
def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    global count_loop_

    # Time of checking leave point will not be a separate time in this algorithm
    if not((state_ == 1 and state == 2) or (state_ == 2 and state == 1)):
        count_state_time_ = 0
    count_loop_ = 0
    
    # Change the state
    state_ = state

    # Informing user that the state has changed
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    # Differnet states turn on and off different servers(other scripts)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
    if state_ == 3:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)


# Function to calculate distance betweeen two points
def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
    return dist    

# Function to normalize angle
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

# Function of checking reaching points of previous steps
def check_prev_points(position):
    global list_prev_points, len_prev_points
    result = 0
    for i in range(len_prev_points-2):
        if calc_dist_points(position, list_prev_points[i]) < 0.15:
            result += 1
    if result > 0:
        result = True
    else: 
        result = False
    return result

# Function of checking if the robot is stuck
def check_stuck(stuck_points, len_stuck_points):
    result = 0
    for i in range(len_stuck_points - 1):
        if calc_dist_points(stuck_points[i], stuck_points[i+1]) < 0.1:
            result += 1
    if result == 4:
        result = True
    else:
        result = False
    return result
    
# Function for getting information about memory usage 
def process_memory():
    process = psutil.Process(os.getpid())
    mem_info = process.memory_info()
    return mem_info.rss

# Function that creates a string with data about points
def list_to_string(list_points):
    str1 = "["
    for i, p in enumerate(list_points):
        if i != 0:
            str1 += "; "
        str1 += str(p.x)
        str1 += " "
        str1 += str(p.y)
    str1 += "]"
    return str1


def main():
    # Stating global parameters
    global regions_, position_, desired_position_, state_, yaw_, st_position_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global count_state_time_, count_loop_, count_point
    global st_point
    global list_prev_points, len_prev_points

    # Variables for collecting statistics
    timer_hp = datetime.datetime.now()
    OBSTACLES_COUNT = 0
    mem_before = process_memory()
    pid = os.getpid()
    POINTS = []
    sum_yaw = 0

    rospy.init_node('distbug')
    
    # Publisher to change velocities
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Initialize subscribers laser and odometery
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    # Initialize servers for other scripts involved
    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    reset_world =rospy.ServiceProxy('/gazebo/reset_world', Empty)
    

    # Set robot position
    model_state = ModelState()
    model_state.model_name = 'turtlebot3_burger'
    
    # Change state to start
    change_state(3)

    # Variables for step-by-step movement
    prev_step_pos = position_
    step = 0.5
    cur_step_dist = 0
    count_steps = 0
    counter = 0
    vision_radius = 1.0

    # Calculate the yaw to the destination point
    desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = desired_yaw - yaw_
    if math.fabs(err_yaw) > math.pi:
        if err_yaw > 0:
            err_yaw = err_yaw - 2 * math.pi
        else: 
            err_yaw = err_yaw + 2 * math.pi
    degree = err_yaw * 180 / math.pi
    # log = "st angle %.4f" % degree
    # rospy.loginfo(log)

    
    # Point the robot to the destination point
    while not math.fabs(err_yaw) <= math.pi / 90:
        twist_msg = Twist()
        desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
        err_yaw = desired_yaw - yaw_
        if math.fabs(err_yaw) > math.pi:
            if err_yaw > 0:
                err_yaw = err_yaw - 2 * math.pi
            else: 
                err_yaw = err_yaw + 2 * math.pi
        if err_yaw > 0:
            twist_msg.angular.z = 0.7
        else: 
            twist_msg.angular.z = -0.7
        pub.publish(twist_msg)
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
    
    # Initialize going to the point
    change_state(0)

    yaw_before = yaw_

    # Variables to check if the robot is stuck
    stuck_points = []
    len_stuck_points = 0
    stuck_timer = 0

    rate = rospy.Rate(20)
    # Circle to change robot states
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        # Calculating the total turn
        diff = math.fabs(yaw_ - yaw_before)
        if diff > 0:
            sum_yaw += math.fabs(yaw_ + yaw_before)
        else:
            sum_yaw += math.fabs(yaw_before - yaw_)
        yaw_before = yaw_

        
        # Check if robot arived at the dest point
        if math.sqrt((desired_position_.y - position_.y)**2 + (desired_position_.x - position_.x)**2) < 0.15 :
            twist_msg = Twist()
            twist_msg.angular.z = 0
            twist_msg.linear.x = 0
            pub.publish(twist_msg)
            log = "point reached"
            POINTS.append(position_)
            rospy.loginfo(log)
            RESULT_TIME = datetime.datetime.now() - timer_hp
            results_file = open("results.txt", "w+")
            results_file.write("ELAPSED TIME: " + str(RESULT_TIME) + "\n")
            results_file.write("OBSTACLES COUNT: " + str(OBSTACLES_COUNT) + "\n")
            results_file.write("POINTS: " + list_to_string(POINTS) + "\n")
            mem_after = process_memory()
            results_file.write(
                "MEMORY USAGE:  " + str(mem_after - mem_before) + "\n"
            )
            results_file.write("COMPLEXITY: HARD" + "\n")
            results_file.write("CALCULATION TIME: -" + "\n")
            results_file.write("TOTAL TURN: " + str(sum_yaw) + "\n")
            results_file.close()
            os.system("rosnode kill /go_to_point")
            os.system("rosnode kill /wall_follower")
            os.system("rosnode kill /wall_follower_left")
            reset_world()
            os.system("kill "+ str(pid))
        
        # Checking if the robot is stuck
        if count_state_time_ > 19 and stuck_timer == 15: 
            if len_stuck_points < 5:
                len_stuck_points += 1
            else:
                if check_stuck(stuck_points, len_stuck_points):
                    log = "Stuck error"
                    rospy.loginfo(log)
                    RESULT_TIME = datetime.datetime.now() - timer_hp
                    results_file = open("results.txt", "w+")
                    results_file.write("ELAPSED TIME: " + str(RESULT_TIME) + "\n")
                    results_file.write("OBSTACLES COUNT: " + str(OBSTACLES_COUNT) + "\n")
                    results_file.write("POINTS: " + list_to_string(POINTS) + "\n")
                    mem_after = process_memory()
                    results_file.write(
                        "MEMORY USAGE:  " + str(mem_after - mem_before) + "\n"
                    )
                    results_file.write("COMPLEXITY: HARD" + "\n")
                    results_file.write("CALCULATION TIME: -" + "\n")
                    results_file.write("TOTAL TURN: " + str(sum_yaw) + "\n")
                    results_file.close()
                    os.system("rosnode kill /go_to_point")
                    os.system("rosnode kill /wall_follower")
                    os.system("rosnode kill /wall_follower_left")
                    reset_world()
                    os.system("kill "+ str(pid))
                else:
                    stuck_points.pop(0)
            stuck_points.append(position_)
            for i in range(len_stuck_points):
                log = 'stuck_points %d [%.4f;%.4f]' % (i,stuck_points[i].x,stuck_points[i].y)
                rospy.loginfo(log)
            stuck_timer = 0



        # Go to point state
        elif state_ == 0:

            # Check if there is an obstacle forward
            if regions_['front'] > 0 and regions_['front'] < 0.25:
                st_point = position_
                hit_point = position_
                OBSTACLES_COUNT += 1
                change_state(1)        

        
        # Wall following
        elif state_ == 1:

            # Checking the robot reached hit point 
            if count_state_time_ > 20 and calc_dist_points(st_point, position_) < 0.2:
                log = "point cannot be reached"
                rospy.loginfo(log)
                RESULT_TIME = datetime.datetime.now() - timer_hp
                results_file = open("results.txt", "w+")
                results_file.write("ELAPSED TIME: " + str(RESULT_TIME) + "\n")
                results_file.write("OBSTACLES COUNT: " + str(OBSTACLES_COUNT) + "\n")
                results_file.write("POINTS: " + list_to_string(POINTS) + "\n")
                mem_after = process_memory()
                results_file.write(
                    "MEMORY USAGE:  " + str(mem_after - mem_before) + "\n"
                )
                results_file.write("COMPLEXITY: HARD" + "\n")
                results_file.write("CALCULATION TIME: -" + "\n")
                results_file.write("TOTAL TURN: " + str(sum_yaw) + "\n")
                results_file.close()
                os.system("rosnode kill /go_to_point")
                os.system("rosnode kill /wall_follower")
                os.system("rosnode kill /wall_follower_left")
                reset_world()
                os.system("kill "+ str(pid))
            
            # Checking the robot has returned to the point of the previous step
            if count_state_time_ > 30 and check_prev_points(position_):
                log = "point cannot be reached, points of previous steps was reached"
                rospy.loginfo(log)
                RESULT_TIME = datetime.datetime.now() - timer_hp
                results_file = open("results.txt", "w+")
                results_file.write("ELAPSED TIME: " + str(RESULT_TIME) + "\n")
                results_file.write("OBSTACLES COUNT: " + str(OBSTACLES_COUNT) + "\n")
                results_file.write("POINTS: " + list_to_string(POINTS) + "\n")
                mem_after = process_memory()
                results_file.write(
                    "MEMORY USAGE:  " + str(mem_after - mem_before) + "\n"
                )
                results_file.write("COMPLEXITY: HARD" + "\n")
                results_file.write("CALCULATION TIME: -" + "\n")
                results_file.write("TOTAL TURN: " + str(sum_yaw) + "\n")
                results_file.close()
                os.system("rosnode kill /go_to_point")
                os.system("rosnode kill /wall_follower")
                os.system("rosnode kill /wall_follower_left")
                reset_world()
                os.system("kill "+ str(pid))

            # Making step-by-step movements
            cur_step_dist += calc_dist_points(position_, prev_step_pos)
            prev_step_pos = position_
            if cur_step_dist > step:
                counter += 1
                twist_msg = Twist()
                twist_msg.linear.x = 0
                pub.publish(twist_msg)
                srv_client_wall_follower_(False)
                if counter == 25:
                    counter = 0
                    count_steps += 1
                    cur_step_dist = 0

                    if len_prev_points < 5:
                        len_prev_points += 1
                    else:
                        list_prev_points.pop(0)
                    list_prev_points.append(position_)
                    for i in range(len_prev_points):
                        log = 'list_prev_points %d [%.4f;%.4f]' % (i,list_prev_points[i].x,list_prev_points[i].y)
                        rospy.loginfo(log)
                    # Checking leave point state
                    change_state(2)


        # Checking if this point can be identified as leave point, looking in the direction of the finish basically        
        elif state_ == 2:

            # Calculation of the desired degree
            desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = desired_yaw - yaw_
            if math.fabs(err_yaw) > math.pi:
                if err_yaw > 0:
                    err_yaw = err_yaw - 2 * math.pi
                else: 
                    err_yaw = err_yaw + 2 * math.pi
            degree = err_yaw * 180 / math.pi
            
            # Matching the desired degrees of rotation with the robot's touch sensors
            if (degree < 25 and degree >= 0) or (degree > -25 and degree <= 0):
                reg = 'front'
            if degree <= 56 and degree > 25:
                reg = 'fleft'
            if degree <= 102 and degree > 56:
                reg = 'left'
            if degree <= 135 and degree > 102:
                reg = 'bleft'
            if degree <= 180 and degree > 135:
                reg = 'bbleft'
            if degree <= -25 and degree > -56:
                reg = 'fright'
            if degree <= -56 and degree > -102:
                reg = 'right'
            if degree <= -102 and degree > -135:
                reg = 'bright'
            if degree <= -135 and degree >= -180:
                reg = 'bbright'

            log = "dist %s %.4f" % (reg, regions_[reg])
            rospy.loginfo(log)

            # Finding the distance to the nearest obstacle in the direction of the goal
            if regions_[reg] > vision_radius:
                free_dist = vision_radius
            else:
                free_dist = regions_[reg]

            # Condition of leaving obstacle
            if regions_[reg] > 0.23 and ((calc_dist_points(position_, desired_position_) - free_dist <= 0)\
                or (calc_dist_points(position_, desired_position_) - free_dist <= calc_dist_points(hit_point, desired_position_) - step)):
                
                # Point the robot to the destination point
                desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
                err_yaw = desired_yaw - yaw_
                while not math.fabs(err_yaw) <= math.pi / 90:
                    twist_msg = Twist()
                    desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
                    err_yaw = desired_yaw - yaw_
                    if math.fabs(err_yaw) > math.pi:
                        if err_yaw > 0:
                            err_yaw = err_yaw - 2 * math.pi
                        else: 
                            err_yaw = err_yaw + 2 * math.pi
                    if err_yaw > 0:
                        twist_msg.angular.z = 0.7
                    else: 
                        twist_msg.angular.z = -0.7
                    pub.publish(twist_msg)
                    twist_msg.angular.z = 0
                    pub.publish(twist_msg)
                # Go to point state
                change_state(0)
            else:
                # Wall following state
                change_state(1)

        # State and stuck timer 
        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            stuck_timer = stuck_timer + 1 
            count_loop_ = 0

        # Appending points for stats
        count_point = count_point + 1
        if count_point == 25:
            count_point = 0
            POINTS.append(position_)
            
        rate.sleep()

if __name__ == "__main__":
    main()
