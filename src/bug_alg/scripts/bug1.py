#! /usr/bin/env python

# import ros stuff
import rospy

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
import datetime

# initializing parameters
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
srv_client_wall_follower_left_ = None
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
regions_ = None
# states of robot during algorithm
state_desc_ = [
    "Go to point",
    "circumnavigate obstacle",
    "go to closest point",
    "turn to circumnavigate left",
    "left_circumnavigating",
    "turn to point/checking for reachability",
]
state_ = 0
circumnavigate_starting_point_ = Point()
circumnavigate_closest_point_ = Point()
count_state_time_ = 0  # seconds the robot is in a state
count_loop_ = 0
count_point = 0
# 0 - go to point
# 1 - circumnavigate
# 2 - go to closest point
# 3 - turn to circumnavigate left
# 4 - circumnavigate left
# 5 - check reachability


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
    regions_ = {
        "left": min(min(msg.ranges[54:89]), 10),
        "fleft": min(min(msg.ranges[18:53]), 10),
        "front": min(min(min(msg.ranges[0:17]), min(msg.ranges[342:359])), 10),
        "fright": min(min(msg.ranges[306:341]), 10),
        "right": min(min(msg.ranges[270:305]), 10),
        "left45": msg.ranges[45],
    }


# state changer
def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_, srv_client_wall_follower_left_
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
        resp = srv_client_wall_follower_left_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
        resp = srv_client_wall_follower_left_(False)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
        resp = srv_client_wall_follower_left_(False)
    if state_ == 3:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        resp = srv_client_wall_follower_left_(False)
    if state_ == 4:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        resp = srv_client_wall_follower_left_(True)
    if state_ == 5:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        resp = srv_client_wall_follower_left_(False)


# function to calculate distance betweeen two points
def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y) ** 2 + (point1.x - point2.x) ** 2)
    return dist


# function to normalize angle
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


def main():
    sum_yaw = 0
    # stating global parameters
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, srv_client_wall_follower_left_
    global circumnavigate_closest_point_, circumnavigate_starting_point_
    global count_loop_, count_state_time_, count_point
    points = []
    length_to_points = []
    timer_hp = datetime.datetime.now()
    OBSTACLES_COUNT = 0
    POINTS = []
    mem_before = process_memory()
    pid = os.getpid()
    
    # publisher to change velocities
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # initialize script as a node
    rospy.init_node("bug1")

    # initialize subscribers laser and odometery
    sub_laser = rospy.Subscriber("/scan", LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber("/odom", Odometry, clbk_odom)

    # initialize servers for other scripts involved
    rospy.wait_for_service("/go_to_point_switch")
    rospy.wait_for_service("/wall_follower_switch")
    rospy.wait_for_service("/wall_follower_left_switch")
    rospy.wait_for_service("/gazebo/set_model_state")

    srv_client_go_to_point_ = rospy.ServiceProxy("/go_to_point_switch", SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy("/wall_follower_switch", SetBool)
    srv_client_wall_follower_left_ = rospy.ServiceProxy(
        "/wall_follower_left_switch", SetBool
    )
    srv_client_set_model_state = rospy.ServiceProxy(
        "/gazebo/set_model_state", SetModelState
    )
    reset_world =rospy.ServiceProxy('/gazebo/reset_world', Empty)

    # set robot position configure it as burger robot
    model_state = ModelState()
    model_state.model_name = "turtlebot3_burger"

    # calculate the yaw to the destination point
    desired_yaw = math.atan2(
        desired_position_.y - position_.y, desired_position_.x - position_.x
    )
    err_yaw = desired_yaw - yaw_

    pth = 0

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
    change_state(0)
    yaw_before = yaw_

    rate_hz = 20
    rate = rospy.Rate(rate_hz)
    # circle to change robot states
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
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
            < 0.2
        ):
            log = "point reached"
            rospy.loginfo(log)
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
            results_file.write("COMPLEXITY: EASY" + "\n")
            results_file.write("CALCULATION TIME: -" + "\n")
            results_file.write("TOTAL TURN: " + str(sum_yaw) + "\n")
            results_file.close()
            os.system("rosnode kill /go_to_point")
            os.system("rosnode kill /wall_follower")
            os.system("rosnode kill /wall_follower_left")
            reset_world()
            os.system("kill "+ str(pid))
            # go to point state
        elif state_ == 0:
            right = 1
            desired_yaw = math.atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x
            )
            err_yaw = desired_yaw - yaw_
            # check if there is an obstacle forward
            if regions_["front"] > 0.15 and regions_["front"] < 0.3:
                circumnavigate_closest_point_ = position_
                circumnavigate_starting_point_ = position_
                # delete the array of points described as leave points if robot has visited an obstacle before
                points.append(position_)
                length_to_points.append(0)
                prev_point_ = position_
                # start circumnavigating
                OBSTACLES_COUNT += 1
                change_state(1)

        elif state_ == 1:
            # compare only after 5 seconds - need some time to get out of starting_point
            # if robot reaches (is close to) starting point
            if (
                count_state_time_ > 20
                and calc_dist_points(position_, circumnavigate_starting_point_) < 0.2
            ):
                right = 1
                least_path = pth
                # choosing the leave point and the direction
                for i in range(0, num_points):
                    if length_to_points[i] < least_path:
                        circumnavigate_closest_point_ = points[i]
                        least_path = length_to_points[i]
                        right = 1
                    if pth - length_to_points[i] < least_path:
                        circumnavigate_closest_point_ = points[i]
                        least_path = pth - length_to_points[i]
                        right = 0
                # if closest point is back of the robot regarding the direction of circumnaigating start turning back
                if right == 0:
                    twist_msg = Twist()
                    twist_msg.linear.x = 0
                    twist_msg.angular.z = 0
                    pth = 0
                    points.clear()
                    length_to_points.clear()
                    num_points = 0
                    # go to state of turning back
                    change_state(3)
                elif right == 1:
                    twist_msg = Twist()
                    twist_msg.linear.x = 0
                    pth = 0
                    points.clear()
                    length_to_points.clear()
                    num_points = 0
                    change_state(2)

            # calculate path to point(perimeter)
            pth += calc_dist_points(position_, prev_point_)
            prev_point_ = position_
            # if current position is closer to the goal than the previous closest_position, assign current position to closest_point

            if calc_dist_points(position_, desired_position_) < calc_dist_points(
                circumnavigate_closest_point_, desired_position_
            ):
                points.clear()
                length_to_points.clear()
                num_points = 1
                points.append(position_)
                length_to_points.append(pth)
                circumnavigate_closest_point_ = position_
            elif calc_dist_points(position_, desired_position_) == calc_dist_points(
                circumnavigate_closest_point_, desired_position_
            ):
                # if the distance from two leave points is equal than we add both of them to the array
                num_points += 1
                points.append(position_)
                length_to_points.append(pth)

        elif state_ == 2:
            # if robot reaches (is close to) closest point

            if calc_dist_points(position_, circumnavigate_closest_point_) < 0.2:
                change_state(5)

        # face left
        elif state_ == 3:
            while regions_["left45"] > 0.4:
                twist_msg = Twist()
                twist_msg.angular.z = 0.3
                pub.publish(twist_msg)
            change_state(4)

        # circumnavigate left until the closest point is near
        elif state_ == 4:

            if calc_dist_points(position_, circumnavigate_closest_point_) < 0.2:
                change_state(5)

        # check if path from the closest point is clear
        elif state_ == 5:
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
            if regions_["front"] > 0.15 and regions_["front"] < 0.45:
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
                os.system("rosnode kill /wall_follower_left")
                reset_world()
                os.system("kill "+ str(pid))
            change_state(0)

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

