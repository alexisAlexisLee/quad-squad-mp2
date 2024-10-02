import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = False

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0

        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y

        o = currentPose.pose.orientation
        rpy = quaternion_to_euler(o.x, o.y, o.z, o.w)
        yaw = rpy[-1]
        
        vel = math.sqrt(currentPose.twist.linear.x ** 2 + currentPose.twist.linear.y ** 2)

        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        target_velocity = 10

        if len(future_unreached_waypoints) == 0:
            return 0

        if len(future_unreached_waypoints) == 1:
            return 5

        p1_x = curr_x
        p1_y = curr_y

        p2_x = future_unreached_waypoints[0][0]
        p2_y = future_unreached_waypoints[0][1]

        p3_x = future_unreached_waypoints[1][0]
        p3_y = future_unreached_waypoints[1][1]
        

        a = math.sqrt((p3_x - p2_x) ** 2 + (p3_y - p2_y) ** 2)
        b = math.sqrt((p3_x - p1_x) ** 2 + (p3_y - p1_y) ** 2)
        c = math.sqrt((p2_x - p1_x) ** 2 + (p2_y - p1_y) ** 2)
        s = (a + b + c) / 2
        area = math.sqrt(s * (s - a) * (s - b) * (s - c))
        if area == 0:
            curvature_kappa = 0
        else:
            radius = (a * b * c) / (4 * area)
            curvature_kappa = 1 / radius

        
        if curvature_kappa <= 0.005:
            target_velocity = 12
        elif curvature_kappa > 0.005 and curvature_kappa < 0.025:
            target_velocity=-200*curvature_kappa+13
        else:
            target_velocity = 8

        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        ld = math.sqrt((curr_x - target_point[0]) ** 2 + (curr_y - target_point[1]) ** 2)
        # delta = atan((2 * L * sin(alpha)) / ld)
        alpha = math.atan2(target_point[1] - curr_y, target_point[0] - curr_x) - curr_yaw
        target_steering = math.atan((2 * self.L * math.sin(alpha)) / ld)

        ####################### TODO: Your TASK 3 code starts Here #######################
        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz



        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
