#!/usr/bin/env python

######################################################################

from logging import raiseExceptions
import rospy
import sys
import time

import math

from asar_hybrid_tmp.srv import  SuturePoints, SuturePointsResponse
from geometry_msgs.msg import PoseStamped, Point
from std_srvs.srv import Empty, EmptyResponse, SetBool
from asar_hybrid_tmp.msg import GraspAction, GraspGoal, BenchmarkData, ArmSuturePointId, PathFollowerAction
from asar_hybrid_tmp.srv import PlanWithPddl, PlanWithPddlResponse, PlanWithPddlRequest, AddNoise, AddNoiseResponse
from behaviortree_ros.srv import GetBT, GetBTRequest

from std_msgs.msg import Float32
from std_msgs.msg import Int32
import numpy as np

import actionlib

import numpy as np
import datetime
# import threading
import statistics
import random


class LogData:
    __slots__ = ['id', 'success','insertionTime', 'extractionTime']
    def __init__(self,id, insertionTime, extractionTime):
        self.id = id
        self.insertionTime = insertionTime
        self.extractionTime = extractionTime
        self.success = False
        

    def get_total_time(self):
        return self.insertionTime + self.extractionTime

class Benchmark:
    def __init__(self):
        self.pub_log_ = rospy.Publisher("/benchmark_log", BenchmarkData, queue_size=1)

        self.pub_feasible_arm_points = rospy.Publisher("/feasible_arm_points", ArmSuturePointId, queue_size=1)
        self.pose_pub = rospy.Publisher("/unit0/asar/ee/cmd", PoseStamped, queue_size=1)

        # self.logs = {}

        self.grasp_counter1 = rospy.Subscriber("/unit0/grasps_counter", Int32, self.counter1_cb)
        self.grasp_counter2 = rospy.Subscriber("/unit1/grasps_counter", Int32, self.counter2_cb)
        # self.suture_points_tack_service = rospy.Service("/suture_service", SuturePoints, self.callback_track_suture_points)
        # self.suture_path_service = rospy.Service("/suture_path_service", SuturePath, self.callback_suture_path)
        self.reset_service = rospy.Service("/start_benchmark", Empty, self.callback_bm)

        self.bt_tracker_service = rospy.Service("/bt_completion_tracker", SetBool, self.bt_track_cb)
        self.add_noise_service_ = rospy.Service("/add_noise_service", AddNoise, self.noise_cb)
        
        self.client1 = actionlib.SimpleActionClient('/unit0/arm_grasp_server', GraspAction)
        self.client2 = actionlib.SimpleActionClient('/unit1/arm_grasp_server', GraspAction) 

        self.path_client1 = actionlib.SimpleActionClient('/unit0/path_follower_server', PathFollowerAction)
        self.path_client2 = actionlib.SimpleActionClient('/unit1/path_follower_server', PathFollowerAction) 

        self.client1.wait_for_server()
        self.client2.wait_for_server()

        self.path_client1.wait_for_server()
        self.path_client2.wait_for_server()

        self.num_points = 10
        self.feasible_points = []
        # self.feasible_points_arm0 = []
        # self.feasible_points_arm1 = []
        self.log_data = []
        self.ompl_timeout = 5.0
        self.num_samples = 10
        self.num_waypoints = 10
        self.mid_point = Point(0.225,0.44,0.087)
        self.radius = 0.012
        self.depth = 0.035
        self.samples_numbers_list = [2,4,6,8,10,12,14,16,18,20,22, 24]
        # self.samples_numbers_list = [2,4,8,12]
        self.ompl_timeout_list = [1.0, 2.0, 3.0, 4.0, 5.0]
        self.num_waypoints_list = [2,4,6,8,10,12,14,16,18]
        self.noise_list = [5,10,15,20,25,30,35,45]
        self.noise_deg = 15
        # self.num_waypoints_list = [2,4,6]
        # self.num_points_list = [2,3,4]
        # self.mid_points_list = [Point(0.20,0.44,0.095), Point(0.22,0.44,0.095), Point(0.24,0.44,0.095),Point(0.20,0.42,0.095), Point(0.20,0.46,0.095), Point(0.24,0.42,0.095),Point(0.24,0.46,0.095)]
        # self.mid_points_list = [Point(0.20,0.42,0.095), Point(0.24,0.44,0.095),Point(0.24,0.42,0.095),Point(0.24,0.46,0.095),Point(0.22,0.44,0.095)]
        self.mid_points_list = [Point(0.22,0.44,0.095), Point(0.24,0.42,0.095),Point(0.24,0.46,0.095)]
        # self.mid_points_list = [Point(0.22,0.44,0.095)]
        self.set_points(self.num_points)
        self.gCount1 = 0
        self.gCount2 = 0
        try:
            reset_srv = rospy.ServiceProxy('/reset_task', Empty)
            reset = reset_srv()
            rospy.sleep(1.0)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
        rospy.logwarn("Benchmark node initialized")
        self.bt_completed = False
        self.bt_success = False

        self.pub_noise_1 = rospy.Publisher("/arm0/set_noise", Float32, queue_size=1)
        self.pub_noise_2 = rospy.Publisher("/arm1/set_noise", Float32, queue_size=1)

    def add_random_noise(self, arm_id):
        if self.noise_deg == 0:
            rospy.loginfo("no noise ")
            return
        noise_range_deg = self.noise_deg
        rospy.logwarn("add noise called ...")
        # Convert noise range from degrees to radians
        

        # Generate a random angle within the noise range
        standard_deviation = noise_range_deg/2 
        random_angle = random.gauss(noise_range_deg, standard_deviation)
        print("noise = ", random_angle)
        # random_angle = random.uniform(-noise_range_deg, noise_range_deg)
        rad = math.radians(random_angle)
        msg = Float32()
        msg.data = rad
        if arm_id == "unit_0":
            self.pub_noise_1.publish(msg)
        elif arm_id == "unit_1":
            # print("arm 2 is adding noise ...")
            self.pub_noise_2.publish(msg)
        else:
            raiseExceptions("arm_id is wrong...")
    def noise_cb(self, req):
        res = AddNoiseResponse()
        arm_id = req.arm_id
        self.add_random_noise(arm_id)
        rospy.sleep(0.1)
        return res

    def bt_track_cb(self,req):
        self.bt_completed = True
        self.bt_success = req.data
        return True, "ok"
    
    def counter1_cb(self, msg):
        self.gCount1 = msg.data

    def counter2_cb(self, msg):
        self.gCount2 = msg.data

    def send_goal_to_arm(self, goal, arm_id):
        if arm_id == 0:
            self.client1.send_goal(goal)
            self.client1.wait_for_result()
            res = self.client1.get_result()
            return res.success
        if arm_id == 1:
            self.client2.send_goal(goal)
            self.client2.wait_for_result()
            res = self.client2.get_result()
            return res.success
        else:
            rospy.logerr("Unknown arm id: %d", arm_id)
            return
    
    def send_path_to_arm(self, goal, arm_id):
        if arm_id == 0:
            self.path_client1.send_goal(goal)
            self.path_client1.wait_for_result()
            res = self.client1.get_result()
            return res.success
        if arm_id == 1:
            self.path_client2.send_goal(goal)
            self.path_client2.wait_for_result()
            res = self.client2.get_result()
            return res.success
        else:
            rospy.logerr("Unknown arm id: %d", arm_id)
            return

    def callback_bm(self, req):

        res = EmptyResponse()
        self.benchmark_loop()
        return res
    
    def get_data__for_suture_trajectory_graph(self, arm_id):
        self.set_points(self.num_points)
        for i in range(0,5):
            for d in self.num_waypoints_list:
                # set goal
                goal = GraspGoal()
                # goal.command = "grasp"
                goal.id = 21
                goal.num_samples = 10
                goal.ompl_timeout = self.ompl_timeout
                goal.num_waypoints = 4
                # success_in = self.send_goal_to_arm(goal, arm_id)
                rospy.sleep(2.0)
                goal.command = "store_suture_trajectory"
                success_in = self.send_goal_to_arm(goal, arm_id)
                rospy.sleep(2.0)
                rospy.wait_for_service('/reset_task')
                try:
                    reset_srv = rospy.ServiceProxy('/reset_task', Empty)
                    reset = reset_srv()
                    rospy.sleep(2.0)
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s"%e)
                return
            # return
        return

    def plot_waypoints(self, arm_id):
        self.set_points(self.num_points)
        for d in self.feasible_points:
            # set goal
            goal = GraspGoal()
            goal.command = "grasp"
            goal.id = 1
            goal.num_samples = self.num_samples
            goal.ompl_timeout = self.ompl_timeout
            goal.num_waypoints = self.num_waypoints
            success_in = self.send_goal_to_arm(goal, arm_id)
            # goal.command = "align"
            # goal.id = 0
            # goal.num_samples = self.num_samples
            # goal.ompl_timeout = self.ompl_timeout
            # goal.num_waypoints = self.num_waypoints
            # success_in = self.send_goal_to_arm(goal, arm_id)
            rospy.sleep(2.0)
            goal.command = "test_waypoints"
            success_in = self.send_goal_to_arm(goal, arm_id)
            rospy.sleep(2.0)
            return
            # if success_in:
            #     rospy.sleep(2.0)
            #     continue
            # else:
            #     break
        return
    


    def mts(self, id, arm_id):
        goal = GraspGoal()
        goal.id = id
        goal.command = "sampling_test"
        start_time = time.time()
        success_in = self.send_goal_to_arm(goal, arm_id)
        if not success_in:
            rospy.logwarn("first suture failed...")
            return
        goal.command = "handover"
        success_in = self.send_goal_to_arm(goal, arm_id)
        if not success_in:
            rospy.logerr("handover failed ...")
            return
        goal.command = "grasp"
        success_in = self.send_goal_to_arm(goal, 1)
        if not success_in:
            rospy.logwarn("grasp by second arm failed ..")
            return
        goal.command = "release"
        success_in = self.send_goal_to_arm(goal, arm_id)
        goal.id = id + 10
        goal.command = "grasp_n"
        if not success_in:
            rospy.logwarn("regrasp by failed ..")
            return
        success_in = self.send_goal_to_arm(goal, arm_id)
        goal.command = "release"
        success_in = self.send_goal_to_arm(goal, 1)
        
        goal.command = "sampling_test"
        success_in = self.send_goal_to_arm(goal, arm_id)
        if not success_in:
            rospy.logwarn("second suture failed ..")
            return
        goal.command = "handover"
        success_in = self.send_goal_to_arm(goal, arm_id)
        if not success_in:
            rospy.logwarn("second handover failed ..")
            return
        goal.command = "grasp"
        success_in = self.send_goal_to_arm(goal, 1)
        if not success_in:
            rospy.logwarn("second grasp by partner failed ..")
            return
        goal.command = "release"
        success_in = self.send_goal_to_arm(goal, arm_id)
        goal.id = id + 20
        goal.command = "grasp_n"
        success_in = self.send_goal_to_arm(goal, arm_id)
        if not success_in:
            rospy.logwarn("second regrasp failed ..")
            return
        goal.command = "release"
        success_in = self.send_goal_to_arm(goal, 1)
        goal.command = "sampling_test"
        success_in = self.send_goal_to_arm(goal, arm_id)
        if not success_in:
            rospy.logwarn("third suture failed ..")
            return
        goal.command = "handover"
        success_in = self.send_goal_to_arm(goal, arm_id)
        
        goal.command = "grasp"
        success_in = self.send_goal_to_arm(goal, 1)
        goal.command = "release"
        success_in = self.send_goal_to_arm(goal, arm_id)
        goal.id = id + 30
        goal.command = "grasp_n"
        success_in = self.send_goal_to_arm(goal, arm_id)
        goal.command = "release"
        success_in = self.send_goal_to_arm(goal, 1)
        goal.command = "sampling_test"
        success_in = self.send_goal_to_arm(goal, arm_id)
        if not success_in:
            rospy.logwarn("4th suture failed ..")
            return
        # goal.command = "handover"
        # success_in = self.send_goal_to_arm(goal, arm_id)
        # goal.command = "grasp"
        # success_in = self.send_goal_to_arm(goal, 1)
        # goal.command = "release"
        # success_in = self.send_goal_to_arm(goal, arm_id)
        # goal.id = id + 40
        # goal.command = "grasp_n"
        # success_in = self.send_goal_to_arm(goal, arm_id)
        # goal.command = "release"
        # success_in = self.send_goal_to_arm(goal, 1)
        # goal.command = "sampling_test"
        # success_in = self.send_goal_to_arm(goal, arm_id)
        # if not success_in:
        #     rospy.logwarn("5th suture failed ..")
        #     return
        # goal.command = "grasp_n"
        # success_in = self.send_goal_to_arm(goal, arm_id)
        # goal.command = "release"
        # success_in = self.send_goal_to_arm(goal, 1)
        d = time.time() - start_time
        print("d = ", d)

        return
        
    def for_coll_check_viz(self):
        goal = GraspGoal()
        goal.id = 0
        # goal.num_samples = 10
        # goal.ompl_timeout = self.ompl_timeout
        goal.command = "coll_check_viz"
        start_time = time.time()
        success_in = self.send_goal_to_arm(goal, 0)
        return
    
    
    def publish_arm_targets(self,):
        for p in self.feasible_points_arm0:            
            msg = ArmSuturePointId()
            msg.arm_id = 0
            msg.suture_point_id = p
            msg.number_of_suture_points = self.num_points
            self.pub_feasible_arm_points.publish(msg)
        for p in self.feasible_points_arm1:            
            msg = ArmSuturePointId()
            msg.arm_id = 1
            msg.suture_point_id = p
            msg.number_of_suture_points = self.num_points
            self.pub_feasible_arm_points.publish(msg)
        return

    
    def set_points(self, num_points):
        rospy.wait_for_service('/suture_service')
        try:
            suture_srv = rospy.ServiceProxy('/suture_service', SuturePoints)
            resp = suture_srv("mts", 0, num_points, self.mid_point, self.radius, self.depth)
            return
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        return

    def save_log_to_txt(self, arm_id):
        success_count = 0
        insertion_times = []
        extraction_times = []
        total_times = []
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = "log_data_num_samples_{}_date-{}.txt".format(self.num_samples,timestamp)
        file = open(filename, "w")
        if file is None:
            raise Exception("Error opening file: {}".format(filename))
        else:
            file.write("number of suture_points: {}\n".format(self.num_points))
            file.write("number of samples: {}\n".format(self.num_samples))
            file.write("number of waypoints: {}\n".format(self.num_waypoints))
            file.write("ompl timeout: {}\n".format(self.ompl_timeout))
            file.write("arm_id: {}\n".format(arm_id))
            file.write("mid_point: {}\n".format(self.mid_point))
            file.write("distance between suture points: {}\n".format(self.radius))
            file.write("suture depth: {}\n".format(self.depth))

            file.write("id,total_time,insertion_time,extraction_time,arm_id,success\n")
            file.write("\n")
            for data in self.log_data:
                if data["success"]:
                    
                    success_count += 1
                    total_times.append(data["total_time"])
                    extraction_times.append(data["extraction_time"])
                    insertion_times.append(data["insertion_time"])
                    file.write("{}\t{:.3f}\t{:.3f}\t{:.3f}\n".format(
                        data["id"],
                        data["total_time"],
                        data["insertion_time"],
                        data["extraction_time"]
                        
                    ))

            file.write("\n")
            file.write("success rate: {}\n".format(success_count / len(self.log_data)))
            if len(total_times) > 1:
                file.write("insertion time mean: {}\n".format(statistics.mean(insertion_times)))
                file.write("insertion time variance: {}\n".format(statistics.variance(insertion_times)))
                file.write("extraction time mean: {}\n".format(statistics.mean(extraction_times)))
                file.write("extraction time variance: {}\n".format(statistics.variance(extraction_times)))
                file.write("total time mean: {}\n".format(statistics.mean(total_times)))
                file.write("total time variance: {}\n".format(statistics.variance(total_times)))
        rospy.logwarn('Saved log data ...')
        # rospy.sleep(2.0)
    
    def save_pddl_log(self, attempts, data):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = "pddl_log{}_date-{}.txt".format(attempts, timestamp)
        with open(filename, "w") as file:
            file.write("number of suture_points: {}\n".format(self.num_points))
            file.write("number of samples: {}\n".format(self.num_samples))
            file.write("number of waypoints: {}\n".format(self.num_waypoints))
            file.write("ompl timeout: {}\n".format(self.ompl_timeout))
            file.write("mid_point: {}\n".format(self.mid_point))
            file.write("distance between suture points: {}\n".format(self.radius))
            file.write("suture depth: {}\n".format(self.depth))
            file.write("\n")

            total_times = []
            costs = []
            success_count = 0
            for d in data:
                success_count += 1
                total_times.append(d["time"])
                costs.append(d["cost"])
                file.write("{}\t{:.2f}\t{:.3f}\n".format(
                    d["id"],
                    d["cost"],
                    d["time"],
                ))

            file.write("\n")
            file.write("success rate: {}\n".format(success_count / attempts))
            if len(costs) > 1:
                file.write("total time mean: {:.3f}\n".format(statistics.mean(total_times)))
                file.write("total time variance: {:.3f}\n".format(statistics.variance(total_times)))
                file.write("cost mean: {:.2f}\n".format(statistics.mean(costs)))
                file.write("cost variance: {:.2f}\n".format(statistics.variance(costs)))

        rospy.logwarn('Saved pddl log data ...')
    
    def save_bt_log(self, attempts, data):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        # filename = "dbt_benchmark_noise_{}_date-{}.txt".format(self.noise_deg, timestamp)
        filename = "bt_benchmark_log{}_date-{}.txt".format(attempts, timestamp)

        with open(filename, "w") as file:
            file.write("number of suture_points: {}\n".format(self.num_points))
            file.write("number of samples: {}\n".format(self.num_samples))
            file.write("number of waypoints: {}\n".format(self.num_waypoints))
            file.write("ompl timeout: {}\n".format(self.ompl_timeout))
            file.write("mid_point: {}\n".format(self.mid_point))
            file.write("distance between suture points: {}\n".format(self.radius))
            file.write("suture depth: {}\n".format(self.depth))
            file.write("\n")

            total_times = []
            costs = []
            success_count = 0
            for d in data:
                success_count += 1
                total_times.append(d["time"])
                costs.append(d["cost"])
                file.write("{}\t{:.2f}\t{:.3f}\n".format(
                    d["id"],
                    d["cost"],
                    d["time"],
                ))

            file.write("\n")
            file.write("success rate: {}\n".format(success_count / attempts))
            if len(costs) > 1:
                file.write("total time mean: {:.3f}\n".format(statistics.mean(total_times)))
                file.write("total time variance: {:.3f}\n".format(statistics.variance(total_times)))
                file.write("cost mean: {:.2f}\n".format(statistics.mean(costs)))
                file.write("cost variance: {:.2f}\n".format(statistics.variance(costs)))

        rospy.logwarn('Saved bt_log  data ...')

    def save_bm_noise_log(self, attempts, data):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = "bm_with_noise_{}_date-{}.txt".format(self.noise_deg, timestamp)
        with open(filename, "w") as file:
            file.write("Combined execution tamp + bt: \n")
            file.write("noise deg: {}\n".format(self.noise_deg))
            file.write("\n")

            total_times = []
            costs = []
            success_count = 0
            for d in data:
                success_count += 1
                total_times.append(d["time"])
                costs.append(d["cost"])
                file.write("{}\t{:.2f}\t{:.3f}\n".format(
                    d["id"],
                    d["cost"],
                    d["time"],
                ))

            file.write("\n")
            file.write("success rate: {}\n".format(success_count / attempts))
            if len(costs) > 1:
                file.write("total time mean: {:.3f}\n".format(statistics.mean(total_times)))
                file.write("total time variance: {:.3f}\n".format(statistics.variance(total_times)))
                file.write("cost mean: {:.2f}\n".format(statistics.mean(costs)))
                file.write("cost variance: {:.2f}\n".format(statistics.variance(costs)))

        rospy.logwarn('Saved bt_log  data ...')

    def save_data(self, attempts, data):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = "bm_data_log{}_date-{}.txt".format(attempts, timestamp)
        with open(filename, "w") as file:
            file.write("number of suture_points: {}\n".format(self.num_points))
            file.write("number of waypoints: {}\n".format(self.num_waypoints))
            file.write("ompl timeout: {}\n".format(self.ompl_timeout))
            file.write("mid_point: {}\n".format(self.mid_point))
            file.write("\n")

            total_times = []
            costs = []
            success_count = 0
            for d in data:
                success_count += 1
                total_times.append(d["time"])
                file.write("{}\t{:.3f}\n".format(
                    d["samples_num"],
                    d["time"],
                ))

            file.write("\n")
            file.write("success rate: {}\n".format(success_count / attempts))
            if len(total_times) > 1:
                file.write("total time mean: {:.3f}\n".format(statistics.mean(total_times)))
                file.write("total time variance: {:.3f}\n".format(statistics.variance(total_times)))

        rospy.logwarn('Saved bm_data_log  data ...')

    def pddlstream_benchmark(self, attempts):
        print("start pddlstream benchmark ....")
        # self.set_points(self.num_points)
        # attempts = 10
        for i in range(21,30):
            data = []
            for j in range(0,attempts):
                rospy.wait_for_service('/pddlstream_service')
                try:
                    pddl_srv = rospy.ServiceProxy('/pddlstream_service', PlanWithPddl)
                    request = PlanWithPddlRequest()
                    request.init_state = str(10)
                    start_time = time.time()
                    result = pddl_srv(request)
                    d = {}
                    if math.isinf(result.cost):
                        print("Value is infinite")
                        return
                        continue
                    d['time'] = time.time() - start_time
                    d['cost'] = (self.gCount2 + self.gCount1)
                    if d['cost']:
                        d['cost'] = d['cost'] - 1
                    d['id'] = i
                    print(d)
                    data.append(d)
                    return
                    rospy.wait_for_service('/reset_task')
                    try:
                        reset_srv = rospy.ServiceProxy('/reset_task', Empty)
                        reset = reset_srv()
                        rospy.sleep(1.0)
                    except rospy.ServiceException as e:
                        rospy.logerr("Service call failed: %s"%e)
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s"%e)
                    continue
            self.save_pddl_log(attempts, data) 
        print("benchmark completed") 
        return
    
    def bt_benchmark(self, attempts):
        req = GetBTRequest()
        req.type = "new"

        # for i in range(0,self.num_points):
        for i in range(26,30):
            data = []
            for j in range(0,attempts):
                self.bt_completed = False
                req.suture_point_id = str(i)
                rospy.wait_for_service('/get_bt_service')
                try: 
                    bt_srv = rospy.ServiceProxy('/get_bt_service', GetBT)
                    res = bt_srv(req)

                    start_time = time.time()
                    rospy.loginfo("waiting for bt completion ....")
                    while True:
                        
                        rospy.sleep(1.0)
                        if self.bt_completed:
                            rospy.loginfo(" bt execution completed ....")
                            break
                    if self.bt_success:
                        d = {}
                        d['time'] = time.time() - start_time
                        d['cost'] = (self.gCount2 + self.gCount1)
                        d['id'] = i
                        data.append(d)
                        print("d = ", d)
                    rospy.wait_for_service('/reset_task')
                    try:
                        reset_srv = rospy.ServiceProxy('/reset_task', Empty)
                        reset = reset_srv()
                        rospy.sleep(1.0)
                    except rospy.ServiceException as e:
                        rospy.logerr("Service call failed: %s"%e)
                    
                except rospy.ServiceException as e:
                    rospy.logerr("bt service failed ...")
                    return False
            
            self.save_bt_log(attempts, data)
            # return

    def bt_benchmark_with_noise(self, attempts):
        req = GetBTRequest()
        req.type = "new"

        for i in range(20,21):
            noise_l = [20,30]
            for noise in noise_l:
                self.noise_deg = noise   
                data = []
                for j in range(0,attempts):
                    self.bt_completed = False
                    req.suture_point_id = str(10)
                    rospy.wait_for_service('/get_bt_service')
                    try: 
                        bt_srv = rospy.ServiceProxy('/get_bt_service', GetBT)
                        res = bt_srv(req)

                        start_time = time.time()
                        rospy.loginfo("waiting for bt completion ....")
                        while True:
                            
                            rospy.sleep(1.0)
                            if self.bt_completed:
                                rospy.loginfo(" bt execution completed ....")
                                break
                        if self.bt_success:
                            d = {}
                            d['time'] = time.time() - start_time
                            d['cost'] = (self.gCount2 + self.gCount1)
                            d['id'] = self.noise_deg
                            data.append(d)
                            print("d = ", d)
                        rospy.wait_for_service('/reset_task')
                        try:
                            reset_srv = rospy.ServiceProxy('/reset_task', Empty)
                            reset = reset_srv()
                            rospy.sleep(1.0)
                        except rospy.ServiceException as e:
                            rospy.logerr("Service call failed: %s"%e)
                        # return
                        
                    except rospy.ServiceException as e:
                        rospy.logerr("bt service failed ...")
                        return False
                
                self.save_bt_log(attempts, data)
            return

    def pddlstream_bt_combined_benchmark(self, attempts):
        req = GetBTRequest()
        req.type = "from_pddl"

        for i in range(10,23):
            noise_l = [10,30]
            for noise in noise_l:
                self.noise_deg = noise   
                data = []
                for j in range(0,attempts):
                    self.bt_completed = False
                    req.suture_point_id = str(30)
                    rospy.wait_for_service('/get_bt_service')
                    try: 
                        bt_srv = rospy.ServiceProxy('/get_bt_service', GetBT)
                        res = bt_srv(req)

                        start_time = time.time()
                        rospy.loginfo("waiting for bt completion ....")
                        while True:
                            
                            rospy.sleep(1.0)
                            if self.bt_completed:
                                rospy.loginfo(" bt execution completed ....")
                                break
                        if self.bt_success:
                            d = {}
                            d['time'] = time.time() - start_time
                            d['cost'] = (self.gCount2 + self.gCount1)
                            d['id'] = noise
                            data.append(d)
                            print("d = ", d)
                         
                            return
                        
                        rospy.wait_for_service('/reset_task')
                        try:
                            reset_srv = rospy.ServiceProxy('/reset_task', Empty)
                            reset = reset_srv()
                            rospy.sleep(1.0)
                        except rospy.ServiceException as e:
                            rospy.logerr("Service call failed: %s"%e)
                        
                    except rospy.ServiceException as e:
                        rospy.logerr("bt service failed ...")
                        return False
                # return
                
                self.save_bm_noise_log(attempts, data)
            return
        return
    
    def sampling_benchmark(self, attempts, arm_id, goalID):
        self.set_points(self.num_points)
        for s in self.samples_numbers_list:
            data = []
            for i in range(0,attempts):
                # set goal
                goal = GraspGoal()
                goal.id = goalID
                # goal.num_samples = 10
                # goal.ompl_timeout = self.ompl_timeout
                goal.num_waypoints = 10
                rospy.sleep(2.0)
                goal.command = "sampling_test"
                start_time = time.time()
                success_in = self.send_goal_to_arm(goal, arm_id)
                # if success_in:
                #     d = {}
                #     d['time'] = time.time() - start_time
                #     d['samples_num'] = 10
                #     data.append(d)
                #     print("d = ", d)
                # rospy.sleep(2.0)
                goal.command = "handover"
                success_in = self.send_goal_to_arm(goal, arm_id)
                if not success_in:
                    rospy.logerr("handover failed ...")
                    return
                # goal.id = 21
                goal.command = "grasp"
                success_in = self.send_goal_to_arm(goal, 1)
                if not success_in:
                    return
                goal.command = "release"
                success_in = self.send_goal_to_arm(goal, arm_id)
                goal.id = 20
                goal.command = "grasp_n"
                success_in = self.send_goal_to_arm(goal, arm_id)
                goal.command = "release"
                success_in = self.send_goal_to_arm(goal, 1)
                
                goal.command = "sampling_test"
                success_in = self.send_goal_to_arm(goal, arm_id)

                goal.command = "handover"
                success_in = self.send_goal_to_arm(goal, arm_id)
                # goal.id = 21
                goal.command = "grasp"
                success_in = self.send_goal_to_arm(goal, 1)
                goal.command = "release"
                success_in = self.send_goal_to_arm(goal, arm_id)
                goal.id = 30
                goal.command = "grasp_n"
                success_in = self.send_goal_to_arm(goal, arm_id)
                goal.command = "release"
                success_in = self.send_goal_to_arm(goal, 1)
                goal.command = "sampling_test"
                success_in = self.send_goal_to_arm(goal, arm_id)
                
                goal.command = "handover"
                success_in = self.send_goal_to_arm(goal, arm_id)
                # goal.id = 21
                goal.command = "grasp"
                success_in = self.send_goal_to_arm(goal, 1)
                goal.command = "release"
                success_in = self.send_goal_to_arm(goal, arm_id)
                goal.id = 40
                goal.command = "grasp_n"
                success_in = self.send_goal_to_arm(goal, arm_id)
                goal.command = "release"
                success_in = self.send_goal_to_arm(goal, 1)
                goal.command = "sampling_test"
                success_in = self.send_goal_to_arm(goal, arm_id)

                goal.command = "handover"
                success_in = self.send_goal_to_arm(goal, arm_id)
                # goal.id = 21
                goal.command = "grasp"
                success_in = self.send_goal_to_arm(goal, 1)
                goal.command = "release"
                success_in = self.send_goal_to_arm(goal, arm_id)
                goal.id = 50
                goal.command = "grasp_n"
                success_in = self.send_goal_to_arm(goal, arm_id)
                goal.command = "release"
                success_in = self.send_goal_to_arm(goal, 1)
                goal.command = "sampling_test"
                success_in = self.send_goal_to_arm(goal, arm_id)
                goal.command = "grasp_n"
                success_in = self.send_goal_to_arm(goal, arm_id)
                goal.command = "release"
                success_in = self.send_goal_to_arm(goal, 1)
                d = {}
                d['time'] = time.time() - start_time
                print("d = ", d)
                return
                # rospy.wait_for_service('/reset_task')
                # try:
                #     reset_srv = rospy.ServiceProxy('/reset_task', Empty)
                #     reset = reset_srv()
                #     rospy.sleep(2.0)
                # except rospy.ServiceException as e:
                #     rospy.logerr("Service call failed: %s"%e)
            self.save_data(attempts, data)
            # if i > 1:
            #     return
        
        
    def benchmark_loop(self,):
        for i in range(self.num_points):
            d = LogData(i,0,0)
            self.feasible_points.append(d)
        # self.run_benchmark(0)
        # self.plot_waypoints(1)
        # self.get_data__for_suture_trajectory_graph(1)
        # self.sampling_benchmark(10, 0, 10)
        # self.sampling_benchmark(10, 1, 1)

        # self.pddlstream_benchmark(10)
        # self.bt_benchmark(10)
        # self.pddlstream_benchmark(10)
        #         
        self.bt_benchmark_with_noise(3)
        # self.pddlstream_bt_combined_benchmark(1)
        # self.bt_benchmark_with_noise(50)
        # self.for_coll_check_viz()
        # self.mts(20, 0)
        return

def bm_trigger():
    bm = Benchmark()
    rospy.spin()

def myhook():
    rospy.loginfo("nodes shutdown ")
    
if __name__ == "__main__":
    rospy.init_node("benchmark_node")
    bm = Benchmark()
    rospy.spin()
    # t_initializer = threading.Thread(name='benchmark_init_trigger', target=bm_trigger)
    # t_initializer.start()
    # t_benchmark_loop = threading.Thread(name='benchamark_loop', target=bm)
    rospy.loginfo("Ready benchmark handler node.")
    rospy.on_shutdown(myhook)
    