from __future__ import print_function

import sys
from pddlstream.algorithms import instantiate_task
instantiate_task.FD_INSTANTIATE = True
from pddlstream.algorithms import instantiation
instantiation.USE_RELATION = True
from pddlstream.algorithms import refinement
refinement.CONSTRAIN_PLANS = False

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import And, print_solution
from pddlstream.language.stream import DEBUG, SHARED_DEBUG, StreamInfo, PartialInputs

from pddlstream.language.constants import PDDLProblem, read_pddlstream_pair
from pddlstream.utils import read, get_file_path, Profiler
from pddlstream.algorithms.incremental import solve_incremental
sys.path.append( '/home/nir/pddlstream/examples/suture' )

from pddlstream.language.generator import from_gen_fn, from_list_fn, from_test, from_fn
from pddlstream.utils import  str_from_object
from geometry_msgs.msg import Pose

import math
import rospy
import threading
from asar_hybrid_tmp.srv import getGraspSamples, PddlMotions, PddlMotionsRequest, getGraspSamplesRequest
from asar_hybrid_tmp.srv import PlanWithPddl, PlanWithPddlResponse
from asar_hybrid_tmp.msg import PddlAction, PathFollowerAction, PathFollowerGoal
import time
from tf.transformations import quaternion_matrix
import numpy as np
from scipy.spatial.transform import Rotation
import actionlib


#need to generate  BT
from dbt_ros.srv import GetBT, GetBTRequest

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

def create_problem(initial_poses):

    arm = 'a0'
    arm1 = 'a1'
    needle = 'n0'
    l_init = 'init'
    loc = initial_poses[0]
    initial_atoms = [
        ('AtLocation',  needle, l_init),
        ('IsArm' , arm),
        ('IsArm' , arm1),
        ('IsLocation' , l_init),
        ('IsLocation' , loc),
        ('IsNeedle', needle),
        ('HandEmpty', arm),
        ('HandEmpty', arm1),
    ]

    goal_literals = [
        # ('AtPose', 'arm0', goal),
        # ('isHolding', arm1, needle),
        # ('AtLocation', needle, l_s1),
        # ('isInserted', needle, l_s1),
        # ('isSutured', needle, loc),
        # ('AtLocation', needle, loc),
        ('isInserted', needle, loc),
        # ('isHolding', arm1, needle),
        # ('HandEmpty', arm),
    ]
    

    domain_pddl, stream_pddl = read_pddlstream_pair(__file__)
    constant_map = {}
    # #stream_map = DEBUG
    # stream_map = SHARED_DEBUG
    
    # maps the stream.pddl defined functions to real functions
    stream_map = {
        "s-motion": from_fn(
            get_path
        ),
        "s-grasp-traj": from_fn(
            get_path
        ),
        "s-grasp-pose": from_gen_fn(sample_grasp),
        "s-insert-loc": from_test(lambda *args: check_insertion_ik(*args)),
        "s-extract-loc": from_test(lambda *args: check_extraction_ik(*args)),
        # "s-extract-loc": from_gen_fn(check_extraction_ik2),
        "s-reachability-loc": from_test(lambda *args: check_location_reachability(*args)),
        # "s-reachability-loc": from_fn(check_location_reachability),
        "s-hold-motion": from_fn(holding_motion),
        "s-insert-motion": from_fn(insert_motion),
        "s-extract-motion": from_fn(extract_motion),
        'distance': length_fn,
    }

    rospy.logwarn("ready to start problem")
    # rospy.sleep(1)
    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map,
                       initial_atoms, And(*goal_literals))

def get_motions(type_cmd, args):
    # print("inside get motions!")
    request = PddlMotionsRequest()
    request.type = type_cmd
    request.grasp = args[1]
    request.target_pose = args[2]
    request.location_id = args[3]
    if args[0] == 'a0':
        
        rospy.wait_for_service('/unit0/motion_handler_pddl')
        try:
            get_motion_values = rospy.ServiceProxy('/unit0/motion_handler_pddl', PddlMotions)
            resp = get_motion_values(request)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            time.sleep(1)
            return False
    else :
        rospy.wait_for_service('/unit1/motion_handler_pddl')
        try:
            get_motion_values = rospy.ServiceProxy('/unit1/motion_handler_pddl', PddlMotions)
            resp = get_motion_values(request)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            time.sleep(1)
            return False

def insert_motion(arm, grasp, loc):
    type = "insertion"
    t = Pose()
    iD = -1
    if loc != 'init':
        n1, iD = extract_numbers(loc)
    g = grasp['grasp']
    args = [arm, g,t, iD]
    resp = get_motions(type, args)
    if resp.IKReachable:
        d = {}
        d['trajectory'] = resp.combined_path
        
        d['cost'] = resp.pathLength
        return (d, )
        # return ('has_move_hold', )
    else :
        # return ('test',)
        return None

def extract_motion(arm, grasp, loc):
    type = "extraction"
    t = Pose()
    iD = -1
    if loc != 'init':
        n1, iD = extract_numbers(loc)
    g = grasp['grasp']
    args = [arm, g,t, iD]
    resp = get_motions(type, args)
    if resp.IKReachable:
        d = {}
        d['trajectory'] = resp.combined_path
        d['cost'] = resp.pathLength
        return (d, )
        # return ('has_move_hold', )
    else :
        # return ('test',)
        return None

def check_insertion_ik(arm, grasp, loc):
    type = "insert"
    t = Pose()
    iD = -1
    if loc != 'init':
        n1, iD = extract_numbers(loc)
    g = grasp['grasp']
    args = [arm, g,t, iD]
    resp = get_motions(type, args)
    return resp.IKReachable

def check_extraction_ik(arm, grasp, loc):
    type = "extract"
    t = Pose()
    iD = -1
    if loc != 'init':
        n1, iD = extract_numbers(loc)
    g = grasp['grasp']
    args = [arm, g,t, iD]
    resp = get_motions(type, args)
    return resp.IKReachable

def check_extraction_ik2(arm, loc):
    type = "extract_grasp"
    t = Pose()
    iD = -1
    if loc != 'init':
        n1, iD = extract_numbers(loc)
    g = Pose()
    args = [arm, g,t, iD]
    resp = get_motions(type, args)
    for p in resp.reachableGrasps:
        d = {}
        d['grasp'] = p
        yield (d,)
    return resp.IKReachable

def extract_numbers(s):
    parts = s.split('_')
    return 0, int(s)

def check_location_reachability(arm, loc, grasp):
    type = "reachability"
    t = Pose()
    iD = -1
    if loc != 'init':
        # print("------------------")
        # print("loc = ", loc)
        n1, iD = extract_numbers(loc)
    g = grasp['grasp']
    args = [arm, g,t, iD]
    # rospy.loginfo("print args ///")
    # print("args = ", args)
    resp = get_motions(type, args)
    # print("resp = ", resp)
    # time.sleep(1.0)
    return resp.IKReachable

def holding_motion(arm, grasp, loc):
    type = "move_holding"
    t = Pose()
    iD = -1
    if loc != 'init':
        n1, iD = extract_numbers(loc)
    g = grasp['grasp']
    args = [arm, g,t, iD]
    resp = get_motions(type, args)
    if len(resp.path) > 0:
        d = {}
        d['trajectory'] = resp.path
        d['cost'] = resp.pathLength
        # d['cost'] = 0
        return (d, )
        # return ('has_move_hold', )
    else :
        # return ('test',)
        return None

def length_fn(control):
    return control['cost']
    

def get_path(arm, goal):
    type = "move"
    id = 0
    gr = Pose()
    g = goal['grasp']
    print("g = ", g)
    args = [arm, gr, g, 0]
    resp = get_motions(type, args)
    if len(resp.path) > 0:
        d = {}
        d['trajectory'] = resp.path
        d['cost'] = resp.pathLength
        return (d, )
    else :
        return None


def sample_grasp(arm, location):
    print("Waiting for service getGraspSamples")
    iD = -1 
    if location != 'init':
        n1, iD = extract_numbers(location)
    req = getGraspSamplesRequest()
    req.action_type = "grasp"
    req.location = iD
    if arm == 'a0':
        rospy.wait_for_service('/unit0/get_grasp_samples')
        try:
            get_samples = rospy.ServiceProxy('/unit0/get_grasp_samples', getGraspSamples)
            resp = get_samples(req)
            for p in resp.samples:
                d = {}
                d['grasp'] = p
                yield (d,)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
    elif arm == 'a1' :
        rospy.wait_for_service('/unit1/get_grasp_samples')
        try:
            get_samples = rospy.ServiceProxy('/unit1/get_grasp_samples', getGraspSamples)
            resp = get_samples(req)
            for p in resp.samples:
                d = {}
                d['grasp'] = p
                yield (d,)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None

    else :
        rospy.logerr("error input for arm ...") 

def planner_service_cb(request):
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)
    suture_point = request.init_state
    init_poses = []
    init_poses.append(suture_point)
    problem = create_problem(init_poses)
    stream_info = {
        "s-motion": StreamInfo(eager=False),
        "s-grasp-traj": StreamInfo(eager=False),
        "s-grasp-pose": StreamInfo(eager=False),
        "s-insert-motion": StreamInfo(eager=False),
        "s-extract-motion": StreamInfo(eager=False),
        "s-motion": StreamInfo(eager=False),
        # "distance": StreamInfo(eager=False),
        "s-extract-loc": StreamInfo(eager=False),
        "s-insert-loc": StreamInfo(eager=False),
        "s-reachability-loc": StreamInfo(eager=False),
    }

    with Profiler(field='tottime'):
        #solution = solve_serialized(problem, planner='ff-eager', unit_costs=args.unit,
        #                            unit_efforts=True, effort_weight=1, debug=False) # max_planner_time=5,
        # solution = solve(problem, algorithm=args.algorithm, stream_info=stream_info,
        #                  unit_costs=args.unit, planner='max-astar',
        #                  unit_efforts=False, effort_weight=1, debug=False, max_iterations=10) # max_planner_time=5,
        # solution = solve(problem, algorithm=args.algorithm, stream_info=stream_info,
        #                  unit_costs=False, planner='ff-eager',
        #                  unit_efforts=False, effort_weight=1, debug=False, max_iterations=30, max_planner_time=120)
        solution = solve(problem, algorithm='adaptive', stream_info=stream_info,
                         unit_costs=False, planner='max-astar',
                         unit_efforts=False, effort_weight=0.1, debug=False, max_time=300) # max_planner_time=5,
        # solution = solve_incremental(
        #             problem, planner='ff-astar', max_time=100, verbose=False, unit_costs=True,
        #         )
        cost = solution[1]
        
        plan = print_solution(solution)
    resp = PlanWithPddlResponse()
    resp.solved = False
    if plan is not None:
        resp.plan = get_action_values_from_plan(plan)
        resp.solved = True
        resp.cost = cost
    else :
        resp.solved = False
    return resp

def get_action_values_from_plan(plan):
    # rospy.logwarn("mapping plan to  ros output ...")
    ros_plan = []
    for d in plan:
        action = PddlAction()
        action.order = d
        action.name = plan[d][0]['name']
        print("name = ",  action.name)
        
        for ar in plan[d][1]['args']:
            if isinstance(ar, str):
                action.args.append(ar)
            if isinstance(ar, dict):
                first_key = next(iter(ar.keys()))
                if first_key == 'grasp':
                    action.grasp = ar['grasp']
                elif first_key == 'trajectory':
                    if action.name == 'insert_n' or action.name == 'extract_n':
                        # print("action name - ", action.name)
                        action.regrasp_path = ar['trajectory']
                        # print("regrasp_path ~ ", action.regrasp_path)
                    else :
                        action.trajectory = ar['trajectory']
        ros_plan.append(action)
        print(" args:",action.args)
    if len(ros_plan) == len(plan):
        # !!!! the following send the plan directly to asar_controller to execute via path_follower action server.
        # execute_tmp_only(ros_plan)
        return ros_plan
    else:
        rospy.logerr("smth went wrong in action val extraction...")
        pass
def pddl_service_cb():
    s = rospy.Service('/pddlstream_service', PlanWithPddl, planner_service_cb)
    rospy.loginfo("Ready to plan pddl")
    rospy.spin()

def send_goal_to_arm(arm, goal):
    client1 = actionlib.SimpleActionClient('/unit0/path_follower_server', PathFollowerAction)
    client2 = actionlib.SimpleActionClient('/unit1/path_follower_server', PathFollowerAction) 

    client1.wait_for_server()
    client2.wait_for_server()
    if arm == 'a0':
        client1.send_goal(goal)
        client1.wait_for_result()
        res = client1.get_result()
        return res.success
    if arm == 'a1':
        client2.send_goal(goal)
        client2.wait_for_result()
        res = client2.get_result()
        return res.success
    else:
        rospy.logerr("Unknown arm id: %d", arm)
        return
    pass

def execute_tmp_only(plan):
    # need to pass goals to: path_follower_server
    success = False
    for action in plan:
        goal = PathFollowerGoal()
        goal.command = action.name
        goal.path = action.trajectory
        goal.combined_path = action.regrasp_path
        success = send_goal_to_arm(action.args[0], goal)

    if success:
        rospy.loginfo("execution should have succeeded...")

def send_plan_to_bt(plan):
    request = GetBTRequest()
    # for a in plan:
    request.type = "from_pddl"
    request.pddl_plan = plan
    rospy.wait_for_service('/pddl_to_bt')
    try:
        convert_to_bt = rospy.ServiceProxy('/pddl_to_bt', GetBT)
        resp = convert_to_bt(request)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        time.sleep(1)
        return False

def shutdownHook():
    rospy.loginfo("nodes shutdown ")

if __name__ == '__main__':
    rospy.init_node('pddlstream',anonymous=False)
    t_pddl = threading.Thread(name='pddl_thread', target=pddl_service_cb)
    t_pddl.start()
    rospy.spin()

    rospy.on_shutdown(shutdownHook)