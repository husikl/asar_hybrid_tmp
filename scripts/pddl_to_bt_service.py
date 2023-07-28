#!/usr/bin/env python
# from sys import dont_write_bytecode
# from typing import Sequence
# from typing import no_type_check_decorator
# from lxml.builder import E
from lxml import etree
import rospy

from behaviortree_ros.msg import BTExecutorAction, BTExecutorGoal
from behaviortree_ros.srv import GetBT, GetBTResponse, SendTree
from asar_hybrid_tmp.srv import PlanWithPddl, PlanWithPddlResponse, PlanWithPddlRequest

from collections import deque

from itertools import tee
import copy
import threading
import time

global objectsInfo
objectsInfo = None
parallelNodeCreated = False


pick1_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" arm_id="0" command="w3" />'
pick2_xml = '<GraspNeedle server_name="/unit1/arm_grasp_server" arm_id="1" command="w1" />'

move1_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" arm_id="1" command="approach" />'
move2_xml = '<GraspNeedle server_name="/unit1/arm_grasp_server" arm_id="1" command="approach" />'

insert1_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" command="insert" />'
insert2_xml = '<GraspNeedle server_name="/unit1/arm_grasp_server" command="insert" />'

is_inserted_xml ='<isInserted service_name="/suture_service" location="" />'
is_sutured_xml ='<isExtracted service_name="/suture_service" location="" />'
is_close_xml = '<isClose service_name="/suture_service" location=""/>'
is_holding_xml = '<isHolding service_name="/unit0/is_holding" unit_id="unit_0" />'
is_holding_xml2 = '<isHolding service_name="/unit1/is_holding" unit_id="unit_1" />'
is_hand_free_xml = '<IsHandFree service_name="/unit0/is_holding" unit_id="unit_0"/>'
is_hand_free_xml_2 = '<IsHandFree service_name="/unit1/is_holding" unit_id="unit_1"/>'
is_grasp_valid_xml = '<isGraspIKValid service_name="unit0/motion_handler_pddl" location="" unit_id=""/>'

# move_holding_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" command="move_holding" pose="{p}" grasp="{g}" traj="{t}" graspTraj="{gt}"/>'
# grasp_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" command="grasp_n" pose="{p}" grasp="{g}" traj="{t}" graspTraj="{gt}"/>'
# insert_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" command="insert_n" pose="{p}" grasp="{g}" traj="{t}" graspTraj="{gt}" suture_id="0"/>'
# extract_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" command="extract_n" pose="{p}" grasp="{g}" traj="{t}" graspTraj="{gt}" suture_id="0"/>'

move_holding_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" command="move_holding"   timeout="500" suture_id="0" />'
grasp_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" command="grasp_n"   timeout="500"  grasp="{g}" />'
insert_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" command="insert_n"   timeout="500"  suture_id="0"/>'
extract_xml = '<GraspNeedle server_name="/unit0/arm_grasp_server" command="extract_n"   timeout="500" suture_id="0"/>'

move_holding_xml_2 = '<GraspNeedle server_name="/unit1/arm_grasp_server" command="move_holding"   timeout="500" suture_id="0" />'
grasp_xml_2 = '<GraspNeedle server_name="/unit1/arm_grasp_server" command="grasp_n"   timeout="500"  grasp="{g}" />'
insert_xml_2 = '<GraspNeedle server_name="/unit1/arm_grasp_server" command="insert_n"   timeout="500"  suture_id="0"/>'
extract_xml_2 = '<GraspNeedle server_name="/unit1/arm_grasp_server" command="extract_n"   timeout="500"  suture_id="0"/>'

handover_xml_1 = '<GraspNeedle server_name="/unit0/arm_grasp_server"   timeout="500" command="handover"  suture_id="0"/>'
handover_xml_2 = '<GraspNeedle server_name="/unit1/arm_grasp_server" command="handover"   timeout="500"  suture_id="0"/>'

release_xml_1 = '<GraspNeedle server_name="/unit0/arm_grasp_server" command="release"   timeout="500"  suture_id=""/>'
release_xml_2 = '<GraspNeedle server_name="/unit1/arm_grasp_server" command="release"   timeout="500"  suture_id=""/>'




def pose_to_string(pose):
    # Extract position and orientation values
    position = pose.position
    orientation = pose.orientation

    # Convert position and orientation to strings
    position_str = f"{position.x};{position.y};{position.z}"
    orientation_str = f"{orientation.x};{orientation.y};{orientation.z};{orientation.w}"

    # Combine position and orientation strings
    pose_str = f"{position_str};{orientation_str}"

    return pose_str

def pose_to_string2(pose):
    # Extract position and orientation values
    position = pose.position
    orientation = pose.orientation

    # Convert position and orientation to strings
    position_str = f"{position.x},{position.y},{position.z}"
    orientation_str = f"{orientation.x},{orientation.y},{orientation.z},{orientation.w}"

    # Combine position and orientation strings
    pose_str = f"{position_str},{orientation_str}"

    return pose_str

def poses_to_string(poses):
    poses_str = []
    for pose in poses:
        pose_str = pose_to_string2(pose)
        poses_str.append(pose_str)
    poses_combined_str = ";".join(poses_str)
    poses_combined_str += ";"  # Add semicolon at the end
    return poses_combined_str

class ActionTemp:
    __slots__ = ['precond', 'name', 'effects', 'xml', 'cos', 'effects_xml']

    def __init__(self,xml, p, e, a, cos, effects_xml):
        self.precond = p
        self.name =  a
        self.xml = xml
        self.effects =  e
        self.cos = cos
        self.effects_xml = effects_xml

    def to_string(self, property):
        s =""
        for p in property :
            
            s += str(p) 
        return s


class Obstacle:
    __slots__ = ['name', 'position']
    
    def __init__(self, name, position):
        self.name = name
        self.position = position

class Location:
    __slots__ = ['name', 'position', 'detected']
    
    def __init__(self, name, position, detected):
        self.name = name
        self.position = position
        self.detected = detected

class Target:
    __slots__ = ['name', 'position', 'detected']
    
    def __init__(self, name, position, detected):
        self.name = name
        self.position = position
        self.detected = detected


class Constraint:
    __slots__ = ['type', 'xml', 'params']
    
    def __init__(self, type, xml, params):
        self.type = type
        self.xml = xml        
        self.params = params
    def __copy__(self):
        cls = self.__class__
        newobject = cls.__new__(cls)
        newobject.__dict__.update(self.__dict__)
        newobject.params = self.params
        newobject.xml = self.xml
        newobject.type = self.type
        return newobject



        
is_inserted = Constraint('is_inserted', is_inserted_xml,{'object': 0, 'in': 0})
is_sutured = Constraint('is_sutured', is_sutured_xml,{'object': 0, 'out': 0})
is_close = Constraint('is_close', is_close_xml,{'robot': 0, 'to': 0})
is_grasp_valid = Constraint('is_grasp_valid', is_grasp_valid_xml,{'grasp': 0, 'ik': 0})

is_holding1 = Constraint('is_holding1',is_holding_xml, {'object': 0, 'hand1':0})
is_holding2 = Constraint('is_holding2',is_holding_xml2, {'object': 0, 'hand2':0})

is_hand_free1 = Constraint('is_hand_free1',is_hand_free_xml, {'empty': 0, 'hand1':0})
is_hand_free2 = Constraint('is_hand_free2',is_hand_free_xml_2, {'empty': 0, 'hand2':0})



constraints= []
constraints.append(is_inserted)
constraints.append(is_sutured)
constraints.append(is_close)
constraints.append(is_holding1)
constraints.append(is_holding2)
constraints.append(is_hand_free1)
constraints.append(is_hand_free2)
constraints.append(is_grasp_valid)


# approach_to = ActionTemp(approach_xml,[is_path_valid],['robot','to'],'approach', 'pose2d', [is_robot_close_to])
# approach1 = ActionTemp(move1_xml,[],['robot','to'],'approach', 'pose2d', [is_close])

# pick1 = ActionTemp(pick1_xml,[is_hand_free1],['object','hand1'],'pick', 'ob', [is_holding1])


move_holding = ActionTemp(move_holding_xml,[is_holding1,is_grasp_valid],['robot','to'],'move_holding', 'pose2d', [is_close])
# move_holding = ActionTemp(move_holding_xml,[is_holding1],['robot','to'],'move_holding', 'pose2d', [is_close])

grasp_n = ActionTemp(grasp_xml,[is_hand_free1],['object','hand1'],'grasp_n', 'ob', [is_holding1])

insert_n = ActionTemp(insert_xml,[is_close,is_grasp_valid, is_holding1],['object','in'],'insert_n', 'ob', [is_inserted])
extract_n = ActionTemp(extract_xml,[is_inserted],['object','out'],'extract_n', 'ob', [is_sutured])

# move_holding2 = ActionTemp(move_holding_xml_2,[is_holding2,is_grasp_valid],['robot','to'],'move_holding', 'pose2d', [is_close])
move_holding2 = ActionTemp(move_holding_xml_2,[is_holding2,is_grasp_valid],['robot','to'],'move_holding', 'pose2d', [is_close])

grasp_n2 = ActionTemp(grasp_xml_2,[is_hand_free2],['object','hand2'],'grasp_n', 'ob', [is_holding2])

insert_n2 = ActionTemp(insert_xml_2,[is_close,is_grasp_valid, is_holding2],['object','in'],'insert_n', 'ob', [is_inserted])
# insert_n2 = ActionTemp(insert_xml_2,[is_close,is_holding2,is_grasp_valid],['object','in'],'insert_n', 'ob', [is_inserted])
extract_n2 = ActionTemp(extract_xml_2,[is_inserted],['object','out'],'extract_n', 'ob', [is_sutured])

handover1 = ActionTemp(handover_xml_1,[],['grasp','ik'],'handover', 'ob', [is_grasp_valid])
release1 = ActionTemp(release_xml_1,[],['empty','hand1'],'release1', 'ob', [is_hand_free1])
release2 = ActionTemp(release_xml_2,[],['empty','hand2'],'release2', 'ob', [is_hand_free2])
handover2 = ActionTemp(handover_xml_2,[],['grasp','ik'],'handover', 'ob', [is_grasp_valid])


actions = []
actions.append(move_holding)
actions.append(grasp_n)
actions.append(insert_n)
actions.append(extract_n)
actions.append(move_holding2)
actions.append(grasp_n2)
actions.append(insert_n2)
actions.append(extract_n2)
actions.append(release1)
actions.append(release2)
actions.append(handover1)
actions.append(handover2)

# action list known to the converter...
bt_actions = []
bt_actions.append(move_holding)
bt_actions.append(grasp_n)
bt_actions.append(insert_n)
bt_actions.append(extract_n)
bt_actions.append(handover1)
bt_actions.append(handover2)
### -----------------------------------------------------------------------------------------------------


dict = {}

def getXmlFromConditionString(goal_condition):
    global constraints
    goalCondition = etree.fromstring(goal_condition)
    for c in constraints :
        element = etree.fromstring(c.xml)
        if goalCondition.tag == element.tag :
            condition = c
    
    condition.xml = etree.tostring(goalCondition, encoding="unicode", pretty_print=True)
    return goalCondition, condition

def getConditionFromXml(xml):
    global constraints
    condition = None
    # print("xml = ", xml)
    for c in constraints :
        element = etree.fromstring(c.xml)
        if xml.tag == element.tag :
            condition = c
            condition.xml = etree.tostring(xml, encoding="unicode", pretty_print=True)
            # print("condition found type = ", condition.type)
    return condition

def createInitialTree(goal_condition):
    root = etree.Element("root")
    bt = etree.Element("BehaviorTree")
    fallback = etree.Element("Fallback")
    
    
    goalCondition, condition = getXmlFromConditionString(goal_condition)
    sub_bt = get_subtree(condition)

    tree = etree.tostring(sub_bt, encoding="unicode", pretty_print=True)
    
    fallback.append(goalCondition)
    fallback.append(sub_bt)
    
    bt.append(fallback)
    root.append(bt)   
    tree = etree.tostring(root, encoding="unicode", pretty_print=True)
    # print(tree)
    root.clear()
    return tree, True

# Function to find all elements with the given tag name using BFS
def find_elements_with_tag_bfs(root, tag):
    found_elements = []
    queue = deque([root])

    while queue:
        el = queue.popleft()  # Pop the next element
        queue.extend(el)      # Append its children

        if el.tag == tag:
            found_elements.append(el)

    return found_elements

def find_elements_with_tag_dfs(node, tag):
    found_elements = []
    
    if node.tag == tag:
        found_elements.append(node)

    for child in node:
        found_elements.extend(find_elements_with_tag_dfs(child, tag))

    return found_elements

# Function to find the leftmost element from a list of elements
def find_leftmost_element(found_elements):
    if not found_elements:
        return None

    leftmost_element = found_elements[0]
    deepest_element = found_elements[0]
    for element in found_elements:
        # if element in deepest_element.iterancestors():
        #     leftmost_element = element

        if len(element.findall('.//*')) > len(deepest_element.findall('.//*')):
                deepest_element = element
    return deepest_element

def updateTree(tree, goal_condition):
    rospy.logwarn("update for goal condition = %s", goal_condition)
    # print(tree)
    # rospy.sleep(10.0)
    root = etree.fromstring(tree)
    # global dict

    found_elements = find_elements_with_tag_bfs(root, goal_condition)
    leftmost_element = find_leftmost_element(found_elements)


    if leftmost_element is None :
        print("not found ...")
        root.clear()
        return tree, False

    
    index = leftmost_element.getparent().index(leftmost_element)
    
    fallback = etree.Element("Fallback")
    parent = leftmost_element.getparent()
    leftmost_element.getparent().remove(leftmost_element)
    fallback.append(leftmost_element)

    condition = getConditionFromXml(leftmost_element)
    if not condition :
        root.clear()
        rospy.loginfo("not a condition node ...")
        return tree, False
    if condition.type == "is_grasp_valid":
        rospy.loginfo("adding a handover routine ...")

        if leftmost_element.attrib['unit_id'] == "unit_0":
            arm_server = '/unit0/arm_grasp_server'
            partner_arm_server = '/unit1/arm_grasp_server'
        else :
            arm_server = '/unit1/arm_grasp_server'
            partner_arm_server = '/unit0/arm_grasp_server'

        seq = etree.Element("Sequence")

        handover_xml = etree.fromstring(handover_xml_1)
        handover_xml.attrib['server_name'] = arm_server
        handover_xml.attrib['command'] = "handover"
        handover_xml.attrib['suture_id'] = leftmost_element.attrib['location']

        
        partner_grasp_xml = etree.fromstring(grasp_xml)
        partner_grasp_xml.attrib['server_name'] = partner_arm_server
        partner_grasp_xml.attrib['command'] = "grasp"
        partner_grasp_xml.attrib['suture_id'] = leftmost_element.attrib['location']


        release1_xml = etree.fromstring(grasp_xml)
        release1_xml.attrib['server_name'] = arm_server
        release1_xml.attrib['command'] = "release"
        release1_xml.attrib['suture_id'] = leftmost_element.attrib['location']

        regrasp_xml = etree.fromstring(grasp_xml)
        regrasp_xml.attrib['server_name'] = arm_server
        regrasp_xml.attrib['command'] = "grasp_n"
        regrasp_xml.attrib['suture_id'] = leftmost_element.attrib['location']

        release2_xml = etree.fromstring(grasp_xml)
        release2_xml.attrib['server_name'] = partner_arm_server
        release2_xml.attrib['command'] = "release"
        release2_xml.attrib['suture_id'] = leftmost_element.attrib['location']

        # choose which arm
        seq.append(handover_xml)
        seq.append(partner_grasp_xml)
        seq.append(release1_xml)
        seq.append(regrasp_xml)
        seq.append(release2_xml)
        seq.append(leftmost_element)
        sub_bt = seq
    else:
        sub_bt = get_subtree(condition)
    fallback.append(sub_bt)
    
    # Sequence.append(fallback)

    parent.insert(index, fallback)
    tree = etree.tostring(root, encoding="unicode", pretty_print=True)
    root.clear()
    # print(tree)
    # rospy.sleep(10.)
    return tree, True
    



############################################################################################################
def get_subtree(constraint):
    global parallelNodeCreated
    global actions
    bt = None
    # print(constraint.type)
    acts = []
    for action in actions:
        # print('Trying with action', action.name, 'Effects', action.effects, 'Condition fluents', fluent.parameters_dict.keys())
        if set(action.effects).issubset(set(constraint.params.keys())):
            
            # print('The action ', action.name, ' can hold ', fluent.name)
            bt = etree.Element("Sequence")
            action_xml = set_action_values_from_condition(action, constraint)
            for c in action.precond:
                c_xml = set_condition_values_from_action(c, action_xml)
                bt.append(c_xml)
            
            bt.append(action_xml)
            acts.append(bt)
    if len(acts) > 1:
        fb = etree.Element("Fallback")
        for b in acts:
            fb.append(b)
        rospy.logwarn("multiple actions ...")
        return fb
    if bt is None:
        raise Exception('Cannot find action with effects', constraint.params.keys())
    return bt


def set_action_values_from_condition(action, condition):
    global objectsInfo
    action_xml = etree.fromstring(action.xml)
    c = etree.fromstring(condition.xml)
   
    if 'location' in c.attrib :
        action_xml.attrib['suture_id'] = c.attrib['location']
    if 'unit_id' in c.attrib:
        if c.attrib['unit_id'] == 'unit_0':
            action_xml.attrib['server_name'] = '/unit0/arm_grasp_server'
        else:    
            action_xml.attrib['server_name'] = '/unit1/arm_grasp_server'
    return action_xml

def set_condition_values_from_action(condition, action_xml):
    global actions, objectsInfo
    c = etree.fromstring(condition.xml)
    action = None
    for a in actions :
        xml = etree.fromstring(a.xml)
        if xml.tag == action_xml.tag :
            action = a
    # print("action found = ", action.action)
    if 'location' in c.attrib :
        c.attrib['location'] = action_xml.attrib['suture_id']
    if 'unit_id' in c.attrib:
        if action_xml.attrib['server_name'] == '/unit0/arm_grasp_server':
            c.attrib['unit_id'] = "unit_0"
            if c.tag == "isHolding":
                c.attrib['service_name']  = '/unit0/is_holding'
            if c.tag == "IsHandFree":
                c.attrib['service_name']  = '/unit0/is_holding'
            #     print("double check 2: c.c.attrib['service_name'] = ", c.attrib['service_name'])
            if c.tag == "isGraspIKValid":
                c.attrib['service_name']  = '/unit0/motion_handler_pddl'
        else:
            c.attrib['unit_id'] = "unit_1"
            if c.tag == "isHolding":
                c.attrib['service_name']  = '/unit1/is_holding'
            if c.tag == "IsHandFree":
                c.attrib['service_name']  = '/unit1/is_holding'
            if c.tag == "isGraspIKValid":
                c.attrib['service_name']  = '/unit1/motion_handler_pddl'
    
    return c

############################################################################################################

def convert(lst):
    return ' '.join(lst).split()

global converted_tree, actionSubtreeDict
converted_tree = None

actionSubtreeDict = {}

def treeFromPddlPlan(req):
    rospy.loginfo("get inside tree from pddl")
    rospy.wait_for_service('/pddlstream_service')
    try:
        pddl_client = rospy.ServiceProxy('/pddlstream_service', PlanWithPddl)
        request = PlanWithPddlRequest()
        request.init_state = req.suture_point_id
        ros_plan = pddl_client(request)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        time.sleep(1)
        return 
    tree = None
    # get_objects_info()
    reset_all()
    root = etree.Element("root")
    tree = createTreeFromPddl(ros_plan.plan)    
    root.append(tree)   

    test_tree = etree.tostring(root, encoding="unicode", pretty_print=True)
    
    print(test_tree)
    rospy.logwarn("waitinf for bt executor...")
    rospy.wait_for_service('/send_tree')
    try:
        rospy.logwarn("sending the tree ...")
        bt = rospy.ServiceProxy('/send_tree', SendTree)
        response = bt(test_tree)
        print(response)
        rospy.logwarn("tree sent?")
        root.clear()

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    return test_tree, True

def getEffects_new(action_template, pddl_action):
    effects = []
    for e in action_template.effects_xml:
        # print("objects =  ", objects)
        e_xml = etree.fromstring(e.xml)
        if 'pose' in e_xml.attrib :
            e_xml.attrib['pose'] = pose_to_string(pddl_action.pose)
        if 'grasp' in e_xml.attrib :
            e_xml.attrib['grasp'] = pose_to_string(pddl_action.grasp)
        if 'location' in e_xml.attrib :
            e_xml.attrib['location'] = pddl_action.args[-1]
        effects.append(e_xml)
    return effects

def getActionPrecondtions_new(action_template, pddl_action):
    preconds = []
    for c in action_template.precond:
        c_xml = etree.fromstring(c.xml)
        if 'pose' in c_xml.attrib :
            c_xml.attrib['pose'] = pose_to_string(pddl_action.pose)
        if 'grasp' in c_xml.attrib :
            c_xml.attrib['grasp'] = pose_to_string(pddl_action.grasp)
        if 'location' in c_xml.attrib :
            c_xml.attrib['location'] = pddl_action.args[-1]
        preconds.append(c_xml)
    return preconds


def extract_numbers(s):
    return 0, int(s)

def getActionFromPddl_new(actionTemp, pddl_action):
    # print("getting action for= ",actionTemp.name)
    # print("xml= ",actionTemp.xml)


    action_xml = etree.fromstring(actionTemp.xml)
    action_xml.attrib['command'] = pddl_action.name
    if pddl_action.args[0] == "a0":
        action_xml.attrib['server_name'] = "/unit0/arm_grasp_server"
    else :
        action_xml.attrib['server_name'] = "/unit1/arm_grasp_server"
    

    action_xml.attrib['grasp'] = pose_to_string(pddl_action.grasp)
    # if pddl_action.name == 'grasp_n':
    #     action_xml.attrib['pose'] = pose_to_string(pddl_action.trajectory[-1])
    if pddl_action.name == 'insert_n' or  pddl_action.name == 'extract_n' :
        loc = pddl_action.args[2]
        n, id = extract_numbers(loc)
        action_xml.attrib['suture_id'] = str(id)
    if pddl_action.name == "move_holding":
        loc = pddl_action.args[3]
        n, id = extract_numbers(loc)
        action_xml.attrib['suture_id'] = str(id)
    # action_xml.attrib['traj'] = poses_to_string(pddl_action.trajectory)
        # grasps = []
        # for g in pddl_action.regrasp_path:
        #     grasps.append(g.grasp)
        # action_xml.attrib['graspTraj'] = poses_to_string(grasps)
    return action_xml




def createReactiveSubtree_new(action):
    global actionSubtreeDict
    # rospy.logwarn("before loop..?")
    actionTemp = None
    for a in bt_actions:
        # print ("compare ",a.name, " and ", action.name)
        if a.name == action.name:
            actionTemp = a
            break
    # print("action_list = ", action_list )
    a_xml = getActionFromPddl_new(actionTemp, action)
    retry_node = etree.Element("RetryUntilSuccesful")
    retry_node.attrib['num_attempts'] = '3'
    fallback = etree.Element("Fallback")
    # sequence = etree.Element("ReactiveSequence")
    sequence = etree.Element("Sequence")
    effects = getEffects_new(actionTemp, action)
    precondtions = getActionPrecondtions_new(actionTemp, action)
    for p in precondtions:
        sequence.append(p)
    retry_node.append(a_xml)
    sequence.append(retry_node)
    for e in effects :
        fallback.append(e)
    fallback.append(sequence)
    actionSubtreeDict[action.name]= [fallback, precondtions, effects]

    return fallback




def createTreeFromPddl(plan):
    rospy.loginfo("createTreeFromPddl..")
    global converted_tree, actionSubtreeDict
    # l = zip(plan,plan[1:])
    # global converted_tree
    # tree_list = []
    converted_tree = etree.Element("BehaviorTree")
    for a1 in reversed(plan) :
        print("name = ", a1.name)
        print("args = ", a1.args)
        # print("a1 = ", a1)
        createReactiveSubtree_new(a1)
        
    
    
    converted_tree.append(actionSubtreeDict[next(iter(actionSubtreeDict))][0])
    stitchTheTree()
    # sitchSimplest()
    print("*******************************************************")
    subtree = etree.tostring(converted_tree, encoding="unicode", pretty_print=True)
    print(subtree)
    print("*******************************************************")

    return converted_tree
    # raise NotImplementedError



def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)

def stitchTrees(key, value):
    global converted_tree, actionSubtreeDict
    # print("inside stitch trees!!!")
    queue = deque([converted_tree])
    while queue:
        el = queue.popleft()  # pop next element
        queue.extend(el)      # append its children
        # print(el.tag)
        if el.tag == "Sequence" or el.tag == "Sequence" or el.tag == "ReactiveSequence":
            for c in el.getchildren():
                for v in value[2]:
                    if c.tag == v.tag:
                        el.replace(c,value[0] )
                        break
            
    

    ## so here t1 is final goal, so it's effect is we keep
    # so we need to check the childs of sequence node in t1 and childs of fallback node in t2!
    # compare the childs and stitch the trees by replacing the node
    
    # raise NotImplementedError

def stitchTheTree():
    global actionSubtreeDict
    print("-----------------------------------")
    # for (key1, value1), (key2, value2) in pairwise(actionSubtreeDict.items()):
    for (key1, value1), (key2, value2) in pairwise(actionSubtreeDict.items()):
        stitchTrees(key1,value2)

def sitchSimplest():
    global actionSubtreeDict, converted_tree
    seq = etree.Element("Sequence")
    for k in actionSubtreeDict.keys():
        rospy.logwarn("action name %s", k )
        seq.append(actionSubtreeDict[k][0])
    converted_tree.append(seq)

def reset_all():
    global actionSubtreeDict, converted_tree
    actionSubtreeDict = {}
    converted_tree = None

############################################################################################################


def create_bt(req):
    # print(req.pddl_plan)
    # raise NotImplementedError
    rospy.logwarn("got update request ...")
    if req.type == str("new"):
        rospy.logwarn("got new tree request ...")
        # tree = createInitialTree(req.goal_condition)
        # goal_condition = etree.fromstring(is_sutured_xml)
        goal_condition = etree.fromstring(is_inserted_xml)
        goal_condition.attrib['location'] = req.suture_point_id
        root = etree.Element("root")
        bt = etree.Element("BehaviorTree")
        sequence = etree.Element("Sequence")
        
        sequence.append(goal_condition)
        bt.append(sequence)
        root.append(bt)
        tree = etree.tostring(root, encoding="unicode", pretty_print=True)
        root.clear()
        print(tree)
        rospy.logwarn("waitinf for bt executor...")
        rospy.wait_for_service('/send_tree')
        try:
            rospy.logwarn("sending the tree ...")
            bt = rospy.ServiceProxy('/send_tree', SendTree)
            response = bt(tree)
            # print(response)
            # rospy.logwarn("tree sent?")

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        return tree, True
    
    if req.type == str("update"):
        rospy.logwarn("got update request ...")
        tree = updateTree(req.bt, req.goal_condition)

    if req.type == str("from_pddl"):
        tree = treeFromPddlPlan(req)

    return tree

def convert_to_bt(req):
    rospy.logwarn("got request from pddl ...")
    if req.type == str("from_pddl"):
        tree = treeFromPddlPlan(req.pddl_plan)

    return tree

def create_bts_server():
    s = rospy.Service('/get_bt_service', GetBT, create_bt)
    print("Ready to generate behavior trees")
    rospy.spin()

def pddl_to_bt_server():
    s = rospy.Service('/pddl_to_bt', GetBT, convert_to_bt)
    print("Ready to generate behavior trees")
    rospy.spin()



def myhook():
    rospy.loginfo("nodes shutdown ")

if __name__ == '__main__':
    rospy.init_node('bts_handler',anonymous=False)
    t_bts = threading.Thread(name='bts_update', target=create_bts_server)
    t_pddl = threading.Thread(name='pddl_to_bts', target=pddl_to_bt_server)
    # t_plan_seder = threading.Thread(name='plan_sender', target=send_plan)
    t_bts.start()
    t_pddl.start()
    # t_plan_seder.start()
    rospy.on_shutdown(myhook)