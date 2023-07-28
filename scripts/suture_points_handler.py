#!/usr/bin/env python

######################################################################

# Author: Luis G. Torres, Mark Moll
from logging import raiseExceptions
import rospy
import sys
import time

import math
from math import sqrt
import argparse
from asar_hybrid_tmp.srv import SuturePath, SuturePathResponse
from asar_hybrid_tmp.srv import isPoseReachable, collisionCheck, SuturePoints, SuturePointsResponse
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from std_srvs.srv import Empty, EmptyResponse
from asar_hybrid_tmp.msg import GraspAction, GraspGoal
import numpy as np
from scipy.linalg import norm
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
import actionlib

import numpy as np
import copy

from tf.transformations import quaternion_matrix

def pose_to_numpy(pose):
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    quaternion = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    rotation_matrix = quaternion_matrix(quaternion)[:3, :3]
    return position, rotation_matrix

def numpy_to_pose(position, rotation):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    q = Rotation.from_matrix(rotation).as_quat()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

def point_to_array(point_msg):
    return np.array([point_msg.x, point_msg.y, point_msg.z])



def get_angle_from_center(c, radius, p):
    point = point_to_array(p)
    center = point_to_array(c)
    # Create a vector from the center of the circle to the point on the circumference
    vector_to_point = point - center
    # Normalize the vector
    vector_to_point = vector_to_point / np.linalg.norm(vector_to_point)
    # Create a vector pointing along the positive x-axis of the circle's coordinate system
    x_axis_vector = np.array([radius, 0, 0])
    # Compute the dot product
    dot_product = np.dot(vector_to_point, x_axis_vector)
    # Compute the angle
    angle = math.acos(dot_product)
    return angle



def circle_center(p_1, p_2, p_3):
    p1 = point_to_array(p_1)
    p2 = point_to_array(p_2)
    p3 = point_to_array(p_3)
    
    u1 = p2 - p1
    u1 = u1 / np.linalg.norm(u1)
    w1 = p3 - p1
    w1 = w1 / np.linalg.norm(w1)
    w = np.cross(u1, w1)
    
    v1 = p2 - p1
    v2 = p3 - p1
    v1v1 = np.dot(v1, v1)
    v2v2 = np.dot(v2, v2)
    v1v2 = np.dot(v1, v2)
    
    base = 0.5/(v1v1*v2v2-v1v2*v1v2)
    k1 = base*v2v2*(v1v1-v1v2)
    k2 = base*v1v1*(v2v2-v1v2)
    c = p1 + v1*k1 + v2*k2 # center
    radius = np.linalg.norm(c - p1)
    
    print("radius: ", radius)
    normal = np.cross(v1, v2)
    z_angle = math.atan2(normal[1], normal[0])
    
    # calculate rotation matrix
    r = Rotation.from_euler('z', z_angle, degrees=False)

    # rotate second time to make the needle face down
    r = r * Rotation.from_euler('y', 90, degrees=True)

    # need to return pose and the radius of the center
    q = r.as_quat()
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = c
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose, radius


def create_transformation_matrix(pose):
    translation = np.array([[1, 0, 0, pose.position.x],
                           [0, 1, 0, pose.position.y],
                           [0, 0, 1, pose.position.z],
                           [0, 0, 0, 1]])
    # create rotation matrix from quaternion
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    r = Rotation.from_quat(quat)
    rotation = r.as_matrix()
    rotation = np.pad(rotation, [(0, 1), (0, 1)], mode='constant')
    rotation[3,3] = 1
    transformation_matrix = np.matmul(translation, rotation)
    return transformation_matrix

class SuturePs:
    __slots__ = ['insertionP', 'extractionP', 'inPoint', 'exPoint', 'inserted', 'extracted', 'id', 'angle']
    def __init__(self,inP, exP, id, angle):
        self.inPoint = inP
        self.exPoint =  exP
        self.inserted = False
        self.extracted = False
        self.id = id
        self.angle = angle

    def is_sutured(self):
        return self.inserted and self.extracted

class SutureService:
    def __init__(self):
        self.pub_points = rospy.Publisher("/stitch_points_cmd", PoseArray, queue_size=1)
        
        self.pub_points2 = rospy.Publisher("/stitch_points_cmd2", PoseArray, queue_size=1)
        self.pub_points3 = rospy.Publisher("/stitch_points_cmd3", PoseArray, queue_size=1)
        self.pub_points4 = rospy.Publisher("/stitch_points_cmd4", PoseArray, queue_size=1)
        self.pub_points5 = rospy.Publisher("/stitch_points_cmd5", PoseArray, queue_size=1)
        
        self.pose_pub = rospy.Publisher("/unit0/asar/ee/cmd", PoseStamped, queue_size=1)

        self.needle_pose_pub = rospy.Publisher("/needle_expected_pose", PoseStamped, queue_size=1)

        self.needle_subscriber = rospy.Subscriber("/needle/tip", PoseStamped, self.tip_pose_callback)
        self.suture_points_tack_service = rospy.Service("/suture_service", SuturePoints, self.callback_track_suture_points)
        self.suture_path_service = rospy.Service("/suture_path_service", SuturePath, self.callback_suture_path)
        self.reset_service = rospy.Service("/reset_task", Empty, self.callback_reset)
        self.tip_pose = PoseStamped()
        self.suture_points = {}
        self.suture_points2 = {}
        self.suture_points3 = {}
        self.suture_points4 = {}
        self.suture_points5 = {}
        self.points = {}
        self.mid_point = Point()
        self.mid_point.x = 0.224 #0.22 #0.26
        self.mid_point.y = 0.41 + 0.02
        self.mid_point.z = 0.088


        self.mid2 = copy.deepcopy(self.mid_point)
        self.mid2.y = self.mid2.y + 0.04
        
        self.mid3 = copy.deepcopy(self.mid2)
        # self.mid3.y = self.mid3.y + 0.04
         
        self.mid4 = copy.deepcopy(self.mid2)
        # self.mid4.y = self.mid4.y + 0.06

        self.mid5 = copy.deepcopy(self.mid2)
        # self.mid5.y = self.mid5.y+ 0.08
        # self.mid5.x = self.mid5.x+ 0.005

        
        self.radius = 0.012
        self.depth = 0.035
        self.client1 = actionlib.SimpleActionClient('/unit0/arm_grasp_server', GraspAction)
        self.client2 = actionlib.SimpleActionClient('/unit1/arm_grasp_server', GraspAction) 
        self.client1.wait_for_server()
        self.client2.wait_for_server()
        self.store_needle_pose = Pose()

        self.store_arm_poses()
        rospy.loginfo('handler init complete ...')

        self.get_points_on_circle(self.mid_point, self.radius, 10)
        self.publish_points_for_mts()
    
    def publish_points_for_mts(self):
            
        self.get_points_on_circle_for_mts(self.mid5, self.radius, 10, self.suture_points5)

        self.get_points_on_circle_for_mts(self.mid2, self.radius, 10, self.suture_points2)
        
        self.get_points_on_circle_for_mts(self.mid3, self.radius, 10, self.suture_points3)
        
        self.get_points_on_circle_for_mts(self.mid4, self.radius, 10, self.suture_points4)

        self.get_points_on_circle_for_mts(self.mid_point, self.radius, 10, self.suture_points)
        
        p1 = Pose()
        p2 = Pose()
        
        msgAr = PoseArray()
        p1.position = self.suture_points[0].inPoint
        p1.orientation.w = 1
        msgAr.poses.append(p1)
        p2.position = self.suture_points[0].exPoint
        p2.orientation.w = 1
        msgAr.poses.append(p2)
        self.pub_points.publish(msgAr)

        msgAr = PoseArray()
        p1.position = self.suture_points2[0].inPoint
        p1.orientation.w = 1
        msgAr.poses.append(p1)
        p2.position = self.suture_points2[0].exPoint
        
        p2.orientation.w = 1
        msgAr.poses.append(p2)
        self.pub_points2.publish(msgAr)

        msgAr = PoseArray()
        p1.position = self.suture_points3[0].inPoint
        p1.orientation.w = 1
        msgAr.poses.append(p1)
        p2.position = self.suture_points3[0].exPoint
        
        p2.orientation.w = 1
        msgAr.poses.append(p2)
        self.pub_points3.publish(msgAr)
        
        msgAr = PoseArray()
        p1.position = self.suture_points4[0].inPoint
        p1.orientation.w = 1
        msgAr.poses.append(p1)
        p2.position = self.suture_points4[0].exPoint
        
        p2.orientation.w = 1
        msgAr.poses.append(p2)
        self.pub_points4.publish(msgAr)

        msgAr = PoseArray()
        p1.position = self.suture_points5[0].inPoint
        p1.orientation.w = 1
        msgAr.poses.append(p1)
        p2.position = self.suture_points5[0].exPoint
        
        p2.orientation.w = 1
        msgAr.poses.append(p2)
        self.pub_points5.publish(msgAr)


    def store_arm_poses(self):
        # Creates a goal to send to the action server.
        goal = GraspGoal()
        goal.command = "store"
        # Sends the goal to the action server.
        self.client1.send_goal(goal)
        self.client2.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client1.wait_for_result()
        self.client2.wait_for_result()
        # self.store_needle_pose = self.tip_pose.pose
        rospy.loginfo('should be stored')

    def callback_reset(self, req):
        needlePose = PoseStamped()
        
        needlePose.pose.position.x = 0.20
        needlePose.pose.position.y = 0.47
        needlePose.pose.position.z = 0.105
        
        rot = Rotation.from_euler('xyz', [-90, -90, 0], degrees=True)
        needlePose.pose.orientation.x = rot.as_quat()[0]
        needlePose.pose.orientation.y = rot.as_quat()[1]
        needlePose.pose.orientation.z = rot.as_quat()[2]
        needlePose.pose.orientation.w = rot.as_quat()[3]
        # needlePose.pose.orientation = self.store_needle_pose.orientation
        self.needle_pose_pub.publish(needlePose)
        
        # print("needle pose = ", needlePose)
        # Creates a goal to send to the action server.
        goal = GraspGoal()
        goal.command = "reset"
        # Sends the goal to the action server.
        self.client1.send_goal(goal)
        self.client2.send_goal(goal)

        # # Waits for the server to finish performing the action.
        self.client1.wait_for_result()
        self.client2.wait_for_result()
        res = EmptyResponse()
        rospy.sleep(0.5)
        self.needle_pose_pub.publish(needlePose)
        rospy.sleep(0.5)
        self.needle_pose_pub.publish(needlePose)
        rospy.sleep(0.5)
        self.needle_pose_pub.publish(needlePose)
        for k in self.suture_points.keys():
            self.suture_points[k].inserted = False
            self.suture_points[k].extracted = False
        
            self.suture_points2[k].inserted = False
            self.suture_points2[k].extracted = False
        
            self.suture_points3[k].inserted = False
            self.suture_points3[k].extracted = False
        
            self.suture_points4[k].inserted = False
            self.suture_points4[k].extracted = False
        
            self.suture_points5[k].inserted = False
            self.suture_points5[k].extracted = False
        rospy.loginfo("reset completed...")
        return res

    def get_center_pose(self, suture_points):
        p_1 = suture_points.inPoint
        p_2 = suture_points.exPoint
        p_3= Point((p_1.x + p_2.x)/2, (p_1.y + p_2.y)/2, p_1.z - self.depth)

        return circle_center(p_1, p_2, p_3)



    def callback_suture_path(self, req):
        res = SuturePathResponse()

        # else:
        # print("req.id = ", req.id)
        p1 = Pose()
        p2 = Pose()
        msgAr = PoseArray()
        sutureId, key = self.extract_digits(req.id)
        suture_point = self.get_dictionary_value(sutureId, key)
        p1.position = suture_point.inPoint
        p1.orientation.w = 1
        msgAr.poses.append(p1)
        p2.position = suture_point.exPoint
        
        p2.orientation.w = 1
        msgAr.poses.append(p2)
        self.pub_points.publish(msgAr)
        res.p_in = suture_point.inPoint
        res.p_out = suture_point.exPoint
        res.angle = suture_point.angle
        rospy.sleep(0.1)
        return res 
    

    def test_suture_points(self):
        
        rate = rospy.Rate(1)
        for item in self.suture_points.items():
            # print(item[1].inPoint)
            p1 = Pose()
            p2 = Pose()
            msgAr = PoseArray()
            p1.position = item[1].inPoint
            p1.orientation.w = 1
            msgAr.poses.append(p1)
            p2.position = item[1].exPoint
            p2.orientation.w = 1
            msgAr.poses.append(p2)
            
            p_1 = p1.position
            p_2= p2.position
            p_3 = Point((p_1.x + p_2.x)/2, (p_1.y + p_2.y)/2, p_1.z - self.depth)

            # test the needle center point
            pose, r = circle_center(p_1, p_2, p_3)
            p_stamped = PoseStamped()
            p_stamped.header.frame_id = "arm0"
            p_stamped.pose = pose
            self.pub_points.publish(msgAr)
            
            self.pose_pub.publish(p_stamped)
            rate.sleep()
            rate.sleep()
            # return

        # print(len(msgAr.poses))
    def tip_pose_callback(self, msg):
        self.tip_pose = msg
    
    # need this for multi-throw execution only.
    def extract_digits(self, n):
        # print("extarct digits called ...")
        # Check if the input is a two-digit number
        if not (10 <= n < 60):
            print("n =", n)
            raise Exception("Input is not a two-digit number.")

        # rospy.sleep(1.0)
        # Extract the first and second digits
        first_digit = n // 10
        second_digit = n % 10

        # Return the extracted digits
        return first_digit, second_digit

    def get_dictionary_value(self, choice, key):
        # Select the dictionary based on the first integer
        # print("choice = ", choice)
        # print("key ~ ", key)
        # rospy.sleep(1.0)
        if choice == 1:
            selected_dict = self.suture_points
            mid = self.mid_point
        elif choice == 2:
            selected_dict = self.suture_points2
            mid = self.mid2
        elif choice == 3:
            selected_dict = self.suture_points3
            mid = self.mid3
        elif choice == 4:
            selected_dict = self.suture_points4
            mid = self.mid4
        elif choice == 5:
            selected_dict = self.suture_points5
            mid = self.mid5
        else:
            raise ValueError("Invalid choice. Choose a number between 1 and 5.")

        # self.get_points_on_circle_for_mts(mid, self.radius, 20, selected_dict)
        # Retrieve the value from the selected dictionary using the second integer as key
        if key in selected_dict:
            return selected_dict[key]
        else:
            print("Available keys:", list(selected_dict.keys()))
            rospy.sleep(15.0)
            raise KeyError("Key not found in the selected dictionary.")

    def callback_track_suture_points(self, req):
        res = SuturePointsResponse()
        
        # print("req - ", req)
        if req.type == "mts":
            self.publish_points_for_mts()
            return res
        
        # if req.id > 0:


        if req.type == "pub points":
            self.test_suture_points()
            return res

        if req.id == 0:
            print("req type = ", req.type)        
        dictionaryId, key = self.extract_digits(req.id)
        suture_point = self.get_dictionary_value(dictionaryId, key)
        if req.type == "inserted":
            rospy.logwarn("inserted called ...")
            if dictionaryId == 1:
                self.suture_points[key].inserted = True
            elif dictionaryId == 2:
                self.suture_points2[key].inserted = True
                
            elif dictionaryId == 3:
                self.suture_points3[key].inserted = True
                
            elif dictionaryId == 4:
                self.suture_points4[key].inserted = True
                
            elif dictionaryId == 5:
                self.suture_points5[key].inserted = True
            return res
        if req.type == "extracted":
            if dictionaryId == 1:
                self.suture_points[key].extracted = True
            elif dictionaryId == 2:
                self.suture_points2[key].extracted = True
                
            elif dictionaryId == 3:
                self.suture_points3[key].extracted = True
                
            elif dictionaryId == 4:
                self.suture_points4[key].extracted = True
                
            elif dictionaryId == 5:
                self.suture_points5[key].extracted = True
            return res
        if req.type == "is_close":
            # rospy.logwarn("is close called ...")
            p1  = point_to_array(suture_point.inPoint)
            p2  = point_to_array(suture_point.exPoint)
            tip_pos, tip_or = pose_to_numpy(self.tip_pose.pose)
            distance_p1 = np.linalg.norm(p1 - tip_pos)
            distance_p2 = np.linalg.norm(p2 - tip_pos)
            
            if distance_p1 <= 0.005 or distance_p2 <= 0.005:  # 5mm = 0.005m
                # rospy.logerr(" it is  close !")
                rospy.sleep(0.1)
                res.close_to = True
            else:
                # rospy.logerr(" not close !")
                res.close_to = False
                rospy.sleep(0.1)
            
        if req.type == "is_extracted" :
            # rospy.logwarn("is extracted called ...")
            res.extracted = suture_point.is_sutured()
        if req.type == "is_inserted" :
            # rospy.logwarn("is inserted called ...")
            # print("is inserted = ", suture_point.inserted)
            if suture_point.inserted :
                rospy.loginfo("inserted .!!!")
            res.inserted = suture_point.inserted
            

        return res
        

    def get_points_on_circle(self, center, radius, num_points):
        for i in range(int(num_points/2)):
            p1 = Point()
            p2 = Point()
            angle = 2 * math.pi * i / num_points
            p1.x = center.x + radius * math.cos(angle)
            p1.y = center.y + radius * math.sin(angle)
            p1.z = center.z
            # print("1 angle = ", angle)
            angle2 = angle + math.pi
            p2.x = center.x + radius * math.cos(angle2)
            p2.y = center.y + radius * math.sin(angle2)
            p2.z = center.z
            # print("2 angle = ", angle2)
            # print("begin")
            # print("p1 = ", p1)
            # print("p2 = ", p2)
            # print("end")
        
            self.suture_points[i] = SuturePs(p1, p2, i, angle)
    
    def get_points_on_circle_for_mts(self, center, radius, num_points, points_dict):
        for i in range(int(num_points/2)):
            p1 = Point()
            p2 = Point()
            angle = 2 * math.pi * i / num_points
            p1.x = center.x + radius * math.cos(angle)
            p1.y = center.y + radius * math.sin(angle)
            p1.z = center.z
            # print("1 angle = ", angle)
            angle2 = angle + math.pi
            p2.x = center.x + radius * math.cos(angle2)
            p2.y = center.y + radius * math.sin(angle2)
            p2.z = center.z
 
        
            points_dict[i] = SuturePs(p1, p2, i, angle)
        

        
if __name__ == "__main__":
    rospy.init_node("suture_points_handler")
    srv = SutureService()
    rospy.loginfo("Ready suture_points_handler node.")
    rospy.spin()