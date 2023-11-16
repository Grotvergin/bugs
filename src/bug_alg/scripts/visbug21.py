#! /usr/bin/env python

# import ros stuff
import rospy
import datetime

# import ros message
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

# import ros service
from std_srvs.srv import *
import os
import psutil
import math
import numpy as np

# initializing parameters
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
return_ = 'NORMAL'
yaw_ = 0
global sum_yaw


yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
# get the innitial position coordinates
position_ = Point()
initial_position_ = Point()
initial_position_.x = rospy.get_param("initial_x")
initial_position_.y = rospy.get_param("initial_y")
initial_position_.z = 0
# get the destination coordinates
desired_position_ = Point()
desired_position_.x = rospy.get_param("des_pos_x")
desired_position_.y = rospy.get_param("des_pos_y")
desired_position_.z = 0

local_target_point = Point()
local_target_point.x = initial_position_.x
local_target_point.y = initial_position_.y
local_target_point.z = initial_position_.z

desired_position_local = Point()
desired_position_local.x = rospy.get_param('des_pos_x')
desired_position_local.y = rospy.get_param('des_pos_y')
desired_position_local.z = 0

point_H = Point()
point_H.x = 0
point_H.y = 0
point_H.z = 0

point_H_check = Point()
point_H_check.x = 0
point_H_check.y = 0
point_H_check.z = 0

point_X = Point()
point_X.x = 0
point_X.y = 0
point_X.z = 0

point_Q = Point()
point_Q.x = 0
point_Q.y = 0
point_Q.z = 0

point_P = Point()
point_P.x = 0
point_P.y = 0
point_P.z = 0

radius = 10

regions_ = None
# states of robot during algorithm
state_desc_ = ["Go to point", "wall following", "checking leave point", "start"]
state_ = 0
count_state_time_ = 0  # seconds the robot is in a state
count_loop_ = 0
count_point = 0
# 0 - go to point
# 1 - wall following
# 2 - checking leave point
# 3 -start position

# callbacks
# robot movement callbacks
def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    )
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


# laser callback
def clbk_laser(msg):
    global regions_
    global angle_increment
    global angle_min
    angle_increment = msg.angle_increment
    angle_min = msg.angle_min
    global left_range_ 
    left_range_ = msg.ranges[54:89]
    global fleft_range_
    fleft_range_ = msg.ranges[18:53]
    global front_range_1
    front_range_1 = msg.ranges[0:10]
    global front_range_2
    front_range_2 = msg.ranges[349:359]
    global fright_range_ 
    fright_range_ = msg.ranges[306:341]
    global right_range_ 
    right_range_ = msg.ranges[270:305]
    regions_ = {
        'left':  min(min(msg.ranges[54:89]), radius),
        'fleft': min(min(msg.ranges[18:53]), radius),
        'front':  min(min(min(msg.ranges[0:10]), min(msg.ranges[349:359])) , radius),
        'fright':  min(min(msg.ranges[306:341]), radius),
        'right':   min(min(msg.ranges[270:305]), radius),
    }


# state changer
def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    # informing user that the state has changed
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    # differnet states turn on and off different servers(other scripts)
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


# fucntion to calculate distance to line connecting start and finish
def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the line
    global st_position_, desired_position_
    p1 = st_position_
    p2 = desired_position_
    up_eq = math.fabs(
        (p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x)
    )
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq

    return distance


# function to calculate distance betweeen two points
def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y) ** 2 + (point1.x - point2.x) ** 2)
    return dist


# funcdion to nprmalize angle
def normalize_angle(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


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



def process_memory():
    process = psutil.Process(os.getpid())
    mem_info = process.memory_info()
    return mem_info.rss


def intersect(p1, p2, p3, p4):
    x1 = p1.x
    y1 = p1.y
    x2 = p2.x
    y2 = p2.y
    x3 = p3.x
    y3 = p3.y
    x4 = p4.x
    y4 = p4.y
    denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
    if denom == 0: # parallel
        return None
    ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom
    if ua < 0 or ua > 1: # out of range
        return None
    ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / denom
    if ub < 0 or ub > 1: # out of range
        return None
    x = x1 + ua * (x2-x1)
    y = y1 + ua * (y2-y1)
    return (x,y)

def cmp(a, b):
    return bool(a > b) - bool(a < b) 

def is_between(a, b, c):
        return ((b.x - a.x) * (c.y - a.y) == (c.x - a.x) * (b.y - a.y) and 
                abs(cmp(a.x, c.x) + cmp(b.x, c.x)) <= 1 and
                abs(cmp(a.y, c.y) + cmp(b.y, c.y)) <= 1)

def isTargetReached(a, b):
    if round(a.x, 0) == round(b.x, 0) and  round(a.y, 0) == round(b.y, 0):
        return True
    else: return False

def step_4():
    if (is_between(initial_position_, desired_position_, local_target_point)):
        point_Q = local_target_point
        step_2()
    else: 
        point_Q = point_X
        return_= 'SUCCESS'

def isVisible():
    local_x = Point()
    local_x.x = 0
    
    if desired_position_.x >= 0:
        local_x.x = position_.x + 10
    else: 
        local_x.x = position_.x - 10
    if regions_['front'] >= radius and local_x.x >= desired_position_.x:
        return True
    elif position_.x + regions_['front'] >= desired_position_.x and local_x.x >= desired_position_.x:
        return True
    else: 
        return False
    
def step_3():
    global point_H_check
    reg = None
    distance_front = 0
    global local_target_point
    
    if regions_['front'] < radius:
        distance_front = regions_['front']
    elif regions_['fleft'] < radius and distance_front<regions_['fleft']:
        distance_front = regions_['fleft']
        reg = 'FLEFT'
    elif regions_['left'] < radius and distance_front<regions_['left']:
        distance_front = regions_['left']
        reg = 'LEFT'
    elif regions_['fright'] < radius and distance_front<regions_['fright']:
        distance_front = regions_['fright']
        reg = 'FRIGHT'
    elif regions_['right'] < radius and distance_front<regions_['right']:
        distance_front = regions_['right']
        reg = 'RIGHT'
    else: distance_front = 0
    
    angle_Q = angle_increment + (3 * angle_min)
    point_Q.x = distance_front * np.cos(angle_Q)
    point_Q.y = distance_front * np.sin(angle_Q)
    
    if (intersect(initial_position_, desired_position_, local_target_point, point_Q) != None) :
        x_local,y_local = intersect(initial_position_, desired_position_, local_target_point, point_Q)
        point_P.x = x_local
        point_P.y = y_local
        if (calc_dist_points(point_P, desired_position_) < calc_dist_points(point_H, desired_position_)):
            point_X = point_P
            if (intersect(point_P, desired_position_, local_target_point, point_Q) != x_local,y_local):
                point_L = point_P
                local_target_point = point_P
                point_H_check = point_H
    if (point_H_check == point_H and is_between(local_target_point, point_Q, point_H)):
        return_= 'BAD'
    else: 
        local_target_point = point_Q
        step_4()


def step_2():
    distance_along_Mline_from_Ti = regions_['front']
    if distance_along_Mline_from_Ti < radius:
        local_target_point.x, point_H.x, point_X.x = desired_position_.x, desired_position_.x, desired_position_.x
        local_target_point.y, point_H.y, point_X.y = position_.y + distance_along_Mline_from_Ti, position_.y + distance_along_Mline_from_Ti, position_.y + distance_along_Mline_from_Ti
        step_3()
    else:
        local_target_point.x = desired_position_.x
        local_target_point.y = position_.y+radius
        step_4()
        



def computeTi_21():
    global distance_along_Mline_from_Ti, return_, glob_steps
    glob_steps = 0

    center=(initial_position_.x,initial_position_.y) #координата робота
    coordinate=(desired_position_.x,desired_position_.y) #координата таргета
    local_target_point = Point()
    point_H = Point()
    point_X = Point()
    point_Q = Point()
    point_P = Point()
    point_L = Point()
    point_H_check = Point()
    
    #calculate the yaw to the destination point
    desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = desired_yaw - yaw_
    if isVisible():
        local_target_point.x = desired_position_.x
        local_target_point.y = desired_position_.y
        rospy.loginfo("target is visible")
        return_= 'SUCCESS'
        change_state(0)
    elif state_ == 1: 
        rospy.loginfo(local_target_point)
        step_3()
    else : 
        rospy.loginfo(local_target_point)
        step_2()
    if (return_ == 'BAD'):
        log = "point cannot be reached"
        rospy.loginfo(log)
        return
    else:
        local_target_point.x = round(local_target_point.x, 1)
        local_target_point.y = round(local_target_point.y, 1)
        desired_position_local = local_target_point
    if glob_steps == 10:
        rospy.set_param('des_pos_x', float(desired_position_.x))
        rospy.set_param('des_pos_y', float(desired_position_.y))
        glob_steps = 0 
    change_state(0)

def main():
    sum_yaw = 0
    # stating global parameters
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_, st_position_, glob_steps
    global srv_client_go_to_point_, srv_client_wall_follower_
    global count_state_time_, count_loop_, count_point
    global st_point
    glob_steps = 0
    pid = os.getpid()
    rospy.init_node("visbug21")

    # publisher to change velocities
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # initialize subscribers laser and odometery
    sub_laser = rospy.Subscriber("/scan", LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber("/odom", Odometry, clbk_odom)

    # initialize servers for other scripts involved
    rospy.wait_for_service("/go_to_point_switch")
    rospy.wait_for_service("/wall_follower_switch")
    rospy.wait_for_service("/gazebo/set_model_state")

    srv_client_go_to_point_ = rospy.ServiceProxy("/go_to_point_switch", SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy("/wall_follower_switch", SetBool)
    srv_client_set_model_state = rospy.ServiceProxy(
        "/gazebo/set_model_state", SetModelState
    )
    reset_world =rospy.ServiceProxy('/gazebo/reset_world', Empty)
    
    # set robot position
    model_state = ModelState()
    model_state.model_name = "turtlebot3_burger"
    # change state to start
    timer_hp = datetime.datetime.now()
    OBSTACLES_COUNT = 0
    POINTS = []

    mem_before = process_memory()

    change_state(3)
    stuck_error = 0
    stuck_pos = position_

    st_position_ = position_

    # calculate the yaw to the destination point
    desired_yaw = math.atan2(
        desired_position_.y - position_.y, desired_position_.x - position_.x
    )
    err_yaw = desired_yaw - yaw_
    stuck_error = 0
    stuck_pos = position_
    stuck_prev_pos = position_

    # point the robot to the destination point
    while not math.fabs(err_yaw) <= math.pi / 90:
        twist_msg = Twist()
        desired_yaw = math.atan2(
            desired_position_.y - position_.y, desired_position_.x - position_.x
        )
        err_yaw = desired_yaw - yaw_
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
        pub.publish(twist_msg)
        twist_msg.angular.z = 0
        pub.publish(twist_msg)

    # initialize going to the point
    while regions_ == None:
        continue
    computeTi_21()
    yaw_before = yaw_
    rate = rospy.Rate(20)
    # circle to change robot states
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
        # calculate distance to line (s-f)
        glob_steps = glob_steps+1
        distance_position_to_line = distance_to_line(position_)
        diff = math.fabs(yaw_ - yaw_before)
        if diff > 1:
            sum_yaw += math.fabs(yaw_ + yaw_before)
        else:
            sum_yaw += math.fabs(yaw_before - yaw_)
        yaw_before = yaw_

        # check if robot arived at the dest point
        if (
            math.sqrt(
                (desired_position_.y - position_.y) ** 2
                + (desired_position_.x - position_.x) ** 2
            )
            < 0.15
        ):
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
            results_file.write("MEMORY USAGE:  " + str(mem_after - mem_before) + "\n")
            results_file.write("COMPLEXITY: HARD" + "\n")
            results_file.write("CALCULATION TIME: -" + "\n")
            results_file.write("TOTAL TURN: " + str(sum_yaw) + "\n")
            results_file.close()
            os.system("rosnode kill /go_to_point")
            os.system("rosnode kill /wall_follower")
            reset_world()
            os.system("kill "+ str(pid))

        # go to point state
        elif state_ == 0:
            # check if there is an obstacle forward
            if regions_["front"] > 0 and regions_["front"] < 0.25:
                st_point = position_
                OBSTACLES_COUNT += 1
                stuck_error = 0
                change_state(1)
            if count_state_time_ > 20 and count_state_time_ % 20 == 0:
                stuck_pos = position_
                if calc_dist_points(stuck_pos, stuck_prev_pos) < 0.1:
                    stuck_error += 1
                if stuck_error > 5:
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
                    results_file.write("COMPLEXITY: MEDIUM" + "\n")
                    results_file.write("CALCULATION TIME: -" + "\n")
                    results_file.write("TOTAL TURN: " + str(sum_yaw) + "\n")
                    results_file.close()
                    os.system("rosnode kill /go_to_point")
                    os.system("rosnode kill /wall_follower")
                    os.system("rosnode kill /wall_follower_left")
                    reset_world()
                    os.system("kill "+ str(pid))
                stuck_prev_pos = stuck_pos

        # circumnavigate until either line is crossed again or closed loop was made then point cannot be reached
        elif state_ == 1:
            stuck_pos = position_
            if (
                count_state_time_ > 20
                and distance_position_to_line < 0.1
                and calc_dist_points(position_, desired_position_)
                < calc_dist_points(st_point, desired_position_)
            ):
                computeTi_21()
                change_state(2)
            if count_state_time_ > 20 and count_state_time_ % 50 == 0:
                stuck_pos = position_
                if calc_dist_points(stuck_pos, stuck_prev_pos) < 0.1:
                    stuck_error += 1
                    rospy.loginfo("stuck error")
                    rospy.loginfo(stuck_error)
                if stuck_error > 5:
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
                    results_file.write("COMPLEXITY: MEDIUM" + "\n")
                    results_file.write("CALCULATION TIME: -" + "\n")
                    results_file.write("TOTAL TURN: " + str(sum_yaw) + "\n")
                    results_file.close()
                    os.system("rosnode kill /go_to_point")
                    os.system("rosnode kill /wall_follower")
                    os.system("rosnode kill /wall_follower_left")
                    reset_world()
                    os.system("kill "+ str(pid))
                stuck_prev_pos = stuck_pos

            if (count_state_time_ > 20 and calc_dist_points(st_point, position_) < 0.1) or return_ == 'BAD':
                log = "point cannot be reached"
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
                results_file.write("COMPLEXITY: EASY" + "\n")
                results_file.write("CALCULATION TIME: -" + "\n")
                results_file.write("TOTAL TURN: " + str(sum_yaw) + "\n")
                results_file.close()
                os.system("rosnode kill /go_to_point")
                os.system("rosnode kill /wall_follower")
                reset_world()
                os.system("kill "+ str(pid))

        # checking if this point can be identified as leave point, looking in the direction of the finish basically
        elif state_ == 2:

            desired_yaw = math.atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x
            )
            err_yaw = desired_yaw - yaw_
            while not math.fabs(err_yaw) <= math.pi / 90:
                twist_msg = Twist()
                desired_yaw = math.atan2(
                    desired_position_.y - position_.y, desired_position_.x - position_.x
                )
                err_yaw = desired_yaw - yaw_
                twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
                pub.publish(twist_msg)
                twist_msg.angular.z = 0
                pub.publish(twist_msg)
                diff = math.fabs(yaw_ - yaw_before)
                if diff > 1:
                    sum_yaw += math.fabs(yaw_ + yaw_before)
                else:
                    sum_yaw += math.fabs(yaw_before - yaw_)
                yaw_before = yaw_

            # when we turned to finish and theres an obstacle we continue to circumnavigate obstacle
            if regions_["front"] > 0.1 and regions_["front"] < 0.25:
                log = "not the leave point, continuing circumnavigating"
                rospy.loginfo(log)
                computeTi_21()
                change_state(1)
            else:
                # if there is no obstacle right in front of the robot then we go towards the next obstacle
                computeTi_21()

        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0
            
        count_point = count_point + 1
        if count_point == 25:
        	count_point = 0
        	POINTS.append(position_)
        rate.sleep()


if __name__ == "__main__":
    main()

