#!/usr/bin/env python

import rospy
import ompl
from ompl import base as ob
from ompl import geometric as og
from asar_hybrid_tmp.srv import getPath, getPathResponse
from asar_hybrid_tmp.srv import isPoseReachable, collisionCheck
from geometry_msgs.msg import Pose

import numpy as np

class AITMotionPlanner:
    def __init__(self):
        self.service1 = rospy.Service("ompl_planner_service", getPath, self.handle_get_path_request)
        self.service2 = rospy.Service("ompl_planner_service_waypoints", getPath, self.get_path_server_callback)

        self.use_center_point = False
        self.arm_index = '0'
        self.state_space = ompl.base.SE3StateSpace()
        self.bounds = ompl.base.RealVectorBounds(3)
        self.bounds.setLow(-2.0)
        self.bounds.setHigh(2.0)
        self.state_space.setBounds(self.bounds)
        
        rospy.logwarn("AITstar planner initialized")

    def isIKValid(self, state):
        t_z = state.getZ()
        if t_z < 0.060:
            return False
        pos = Pose()
        pos.position.x = state.getX()
        pos.position.y = state.getY()
        pos.position.z = state.getZ()
        pos.orientation.x = state.rotation().x
        pos.orientation.y = state.rotation().y
        pos.orientation.z = state.rotation().z
        pos.orientation.w = state.rotation().w
        resp = self.check_for_IK(pos, self.arm_index)
        if resp.valid:
            valid = self.check_for_Collisions(pos, resp.jointValues, 'unit' + self.arm_index)
            return valid
        return False

    def check_for_IK(self, pose, arm_index):
        service_name = "/unit" + arm_index + "/check_pose_for_ik"
        rospy.wait_for_service(service_name)
        try:
            check_ik = rospy.ServiceProxy(service_name, isPoseReachable)
            response = check_ik(pose)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False

    def check_for_Collisions(self, pose, q, arm_index):
        rospy.wait_for_service("/check_collisions_fcl")
        try:
            check_colls = rospy.ServiceProxy("check_collisions_fcl", collisionCheck)
            resp = check_colls(pose, arm_index, q)
            return resp.collision_free
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False

    def get_path_server_callback(self, req):
        res = getPathResponse()
        # self.use_center_point = False
        # if req.type == "insert":
        #     self.use_center_point = True
        self.arm_index = req.arm_index

        s = ob.State(self.state_space)
        s().setX(req.start.position.x)
        s().setY(req.start.position.y)
        s().setZ(req.start.position.z)
        s().rotation().x = req.start.orientation.x
        s().rotation().y = req.start.orientation.y
        s().rotation().z = req.start.orientation.z
        s().rotation().w = req.start.orientation.w

        self.setup = ompl.geometric.SimpleSetup(self.state_space)
        self.setup.setStateValidityChecker(ob.StateValidityCheckerFn(self.isIKValid))

        intermediate_states = []
        for goal in req.goalWaypoints:
            st = ob.State(self.state_space)
            st().setX(goal.position.x)
            st().setY(goal.position.y)
            st().setZ(goal.position.z)
            st().rotation().x = goal.orientation.x
            st().rotation().y = goal.orientation.y
            st().rotation().z = goal.orientation.z
            st().rotation().w = goal.orientation.w
            if not self.setup.getSpaceInformation().isValid(st.get()):
                print("Intermediate goal is invalid")
                break
            intermediate_states.append(st)
        
        res.max_reachable_waypoint_id = len(intermediate_states) - 1
        return res

    def handle_get_path_request(self, req):
        res = getPathResponse()
        self.arm_index = req.arm_index

        start = ob.State(self.state_space)
        start().setX(req.start.position.x)
        start().setY(req.start.position.y)
        start().setZ(req.start.position.z)
        start().rotation().x = req.start.orientation.x
        start().rotation().y = req.start.orientation.y
        start().rotation().z = req.start.orientation.z
        start().rotation().w = req.start.orientation.w

        goal = ob.State(self.state_space)
        goal().setX(req.goal.position.x)
        goal().setY(req.goal.position.y)
        goal().setZ(req.goal.position.z)
        goal().rotation().x = req.goal.orientation.x
        goal().rotation().y = req.goal.orientation.y
        goal().rotation().z = req.goal.orientation.z
        goal().rotation().w = req.goal.orientation.w

        self.setup = ompl.geometric.SimpleSetup(self.state_space)
        self.setup.setStateValidityChecker(ob.StateValidityCheckerFn(self.isIKValid))

        self.planner = ompl.geometric.AITstar(self.setup.getSpaceInformation())
        self.planner.setUseKNearest(True)
        self.planner.enablePruning(True)
        self.planner.setBatchSize(2)
        # self.planner.setRepairReverseSearch(True)
        self.setup.setStartAndGoalStates(start, goal, 0.00001)
        # self.setup.setStartAndGoalStates(start, goal, 0.001)
        self.setup.setPlanner(self.planner)

        time_out2 = ompl.base.timedPlannerTerminationCondition(req.time_out)
        exact_condition = ompl.base.exactSolnPlannerTerminationCondition(self.setup.getProblemDefinition())
        self.condition = ompl.base.plannerOrTerminationCondition(time_out2, exact_condition)
        solved = self.setup.solve(self.condition)

        speed = req.speed
        # speed = 0.001
        pathLength = 0
        if solved:
            # self.setup.simplifySolution()
            path = self.setup.getSolutionPath()
            simplifier = og.PathSimplifier(self.setup.getSpaceInformation())
            simplifier.simplifyMax(path)
            simplifier.smoothBSpline(path, 1, 0.001)

            resolution = round((path.length()) / speed)
            
            path.interpolate(resolution)
            pathLength = round(path.length(), 3)

            for s in path.getStates():
                p = Pose()
                p.position.x = s.getX()
                p.position.y = s.getY()
                p.position.z = s.getZ()
                p.orientation.x = s.rotation().x
                p.orientation.y = s.rotation().y
                p.orientation.z = s.rotation().z
                p.orientation.w = s.rotation().w
                res.path.append(p)
            
            res.pathLength = pathLength
            p1 = np.array([req.goal.position.x, req.goal.position.y, req.goal.position.z])
            p2 = np.array([res.path[-1].position.x, res.path[-1].position.y, res.path[-1].position.z])
            distance = np.linalg.norm(p1 - p2)
            if distance > 0.002:
                res.path = []
                rospy.logwarn("far from target return no solution...")
                return res
            print("path length is : ", pathLength)
            print("traj size is : ", len(res.path))
            rospy.loginfo("solved ..")
        else:
            rospy.logerr("Not solved by OMPL.")
    
        return res

if __name__ == "__main__":
    rospy.init_node("ompl_planner_node")
    planner = AITMotionPlanner()
    rospy.loginfo("Ready ompl_planner_node.")
    rospy.spin()
