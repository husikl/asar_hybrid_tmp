#!/usr/bin/env python
from logging import raiseExceptions
import hppfcl
import pinocchio as pin
import numpy as np

import itertools
import tf

import rospy
from asar_hybrid_tmp.srv import collisionCheck 
from pinocchio.utils import *
from pinocchio.visualize import RVizVisualizer
import copy
from sensor_msgs.msg import JointState

class CollisionCheck(object):

    def __init__(self, links, capsuleSizes):
        self.links = links
        self.capsules = {}
        caps1 = []
        for l, size in zip(links, capsuleSizes) :
            c = hppfcl.Capsule(size[0], size[1])
            c.name = l
            # print(c.name)
            caps1.append(c)
        self.capsules['unit0'] = caps1
        
        caps2 = []
        for l, size in zip(links, capsuleSizes) :
            c = hppfcl.Capsule(size[0], size[1])
            c.name = l
            # print(c.name)
            caps2.append(c)
        self.capsules['unit1'] = caps2
    
        self.service = rospy.Service('check_collisions_fcl', collisionCheck, self.collision_service_cb)

        ##pinocchio model related
        my_path =  "/home/nir/asar_ws/src/asar_description/urdf/asar_v2_ik_solver.urdf"
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(my_path)
        self.data, self.collision_data, self.visual_data  = pin.createDatas(self.model, self.collision_model, self.visual_model)
        # print('model name: ' + self.model.name)
        
        origin_translation = [0, 0, 0.14]
        # Define the Euler angles
        euler_angles = [0,0,0]

        # Convert Euler angles to quaternion
        quaternion = tf.transformations.quaternion_from_euler(*euler_angles)

        self.tf_between_links = [
            # Predefined transformation between "interface_link" and "link_collision_0"
            (origin_translation, quaternion),
            ([0.0, -0.01, 0.0], [0, 0, 0, 1.0]),
            # Predefined transformation between other links
            # Add more if necessary
        ]
        
        self.sub1 = rospy.Subscriber("/unit0/joint_states", JointState, self.callback_unit0)
        self.sub2 = rospy.Subscriber("/unit1/joint_states", JointState, self.callback_unit1)
        
        self.tf_between_arms = pin.SE3(pin.Quaternion(np.array([0,0,0,1])), np.array([0.0, 0.88, 0.0]) )
        self.debug = False
        rospy.loginfo("service init complete")

    def callback_unit0(self, msg):
        self.unit0_joints = []
        for i in range(9):
            self.unit0_joints.append(msg.position[i])
        # print( " unit 0 ", self.unit0_joints)
    def callback_unit1(self, msg):
        self.unit1_joints = []
        for i in range(9):
            self.unit1_joints.append(msg.position[i])


    def updateCapsulePositionsWithFK(self, capsules, jointVals):
        
        q = np.array(jointVals)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data, q)
        

        for k in range(len(self.collision_model.geometryObjects)):

            oM = self.collision_data.oMg[k]
            if self.collision_model.geometryObjects[k].name == "interface_link_0" :
                interface_link = oM

            elif self.collision_model.geometryObjects[k].name == "finger_left_link_0" :
                fing_left_link = oM            
            else :
                continue            
        linkPoses = [interface_link, fing_left_link ]

        for pose, caps, offset in zip(linkPoses, capsules, self.tf_between_links) :
            quat = pin.Quaternion(np.array(offset[1]))
            trans = np.array(offset[0])
            linkTf = pin.SE3(quat, trans)
            capsPose = pose * linkTf
            if caps.name == "link_collision_1" or caps.name == "link_collision_2"  :
                r_x =  pose.rotation[:, 0]
                r_y =  pose.rotation[:, 1]
                r_z =  pose.rotation[:, 2]
                fixedR = np.eye(3)
                fixedR[:,0] = r_x
                fixedR[:,1] = r_z
                fixedR[:,2] = -r_y

                caps.placement = hppfcl.Transform3f(fixedR, capsPose.translation)
            else :
                caps.placement = hppfcl.Transform3f(pose.rotation, capsPose.translation)

        return
    

    def apply_offset_between_arms(self, capsules, unit_name):
        if unit_name == 'unit0':
            return
        else :
            # print(unit_name)
            # rospy.logwarn("need to apply offset")
            for c in capsules:
                trans = np.array(c.placement.getTranslation())
                rot = np.array(c.placement.getRotation())
                pose = pin.SE3(rot, trans)
                capsPose =  self.tf_between_arms* pose
                # print("capsPose = ", capsPose )
                c.placement = hppfcl.Transform3f(capsPose.rotation, capsPose.translation)
        return
    

    def collision_service_cb(self, req):
        # rospy.loginfo("got request ..")
        # print("arm index = ", req.arm_index)
        collision_free = True
        joints = {}
        joints['unit0'] = self.unit0_joints
        joints['unit1'] = self.unit1_joints
        for k in self.capsules.keys() :
            if k == req.arm_index :
                self.updateCapsulePositionsWithFK(self.capsules[k], req.jointValues)
            else :
                self.updateCapsulePositionsWithFK(self.capsules[k], joints[k])

        for k in self.capsules.keys():
            self.apply_offset_between_arms(self.capsules[k], k) 

        a = self.capsules['unit0']
        b = self.capsules['unit1']
        caps = list(itertools.product(a, b))
        i = 0

        if self.debug :
            

            model = pin.Model()

            geom_model = pin.GeometryModel()
            

            for c in a :
                geom_obj = pin.GeometryObject(c.name, 0, 0, c, c.placement)
                color = np.random.uniform(0, 1, 4)
                color[3] = 1
                geom_obj.meshColor = color
                geom_model.addGeometryObject(geom_obj)


            viz = RVizVisualizer(model, geom_model, geom_model)
            viz.initViewer(initRosNode=False)
            viz.loadViewerModel("pinocchio")

            viz.display(np.zeros(0))

            model2 = pin.Model()

            geom_model2 = pin.GeometryModel()

            for c in b :
                geom_obj = pin.GeometryObject(c.name, 0, 0, c, c.placement)
                color = np.random.uniform(0, 1, 4)
                color[3] = 1
                geom_obj.meshColor = color
                geom_model2.addGeometryObject(geom_obj)

            viz2 = RVizVisualizer(model2, geom_model2, geom_model2)

            # Initialize the viewer.
            viz2.initViewer(viz.viewer, initRosNode=False)
            viz2.loadViewerModel(rootNodeName = "pinocchio2")
            viz2.display(np.zeros(0))

            # viz3 = RVizVisualizer(self.model, self.collision_model, self.visual_model)

            # # Initialize the viewer.
            # viz3.initViewer(viz.viewer, initRosNode=False)
            # viz3.loadViewerModel(rootNodeName = "pinocchio3")
            # viz3.display(np.array(self.unit0_joints))

            # viz4 = RVizVisualizer(self.model, self.collision_model, self.visual_model)

            # # Initialize the viewer.
            # viz4.initViewer(viz.viewer, initRosNode=False)
            # viz4.loadViewerModel(rootNodeName = "pinocchio4")
            # viz4.display(np.array(self.unit1_joints))

            input("Press enter to exit...")
            rospy.sleep(10)

        for val in caps:
            # print(i)
            res = hppfcl.CollisionResult()
            req = hppfcl.CollisionRequest()
            hppfcl.collide(val[0], val[0].placement, val[1], val[1].placement, req, res)
            collision = res.isCollision()
            if collision:
                # print(val[0].name, " " ,val[0].placement.getTranslation())
                # print(val[1].name, " " ,val[1].placement.getTranslation())
                collision_free = False
                break
            i += 1
        if self.debug:
            print("collision_free =~ ", collision_free)
        return collision_free
    
        
if __name__ == '__main__':
    rospy.init_node('fcl_collision_checker')
    
    # size of the capsules s1 is for the shaft and s2 is for end effector fingers
    s1 = (0.0042, 0.284)
    s2 = (0.004, 0.024)

    sizes = [s1, s2]
    ls = ['link_collision_0', 'link_collision_1']
    server = CollisionCheck(ls, sizes)
    rospy.spin()
