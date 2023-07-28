#include <asar_hybrid_tmp/grasp_server.h>

AsarHybridTmp ::AsarHybridTmp(ros::NodeHandle &_node_handle,
                                        bool *           kill_this_node)
    : ac_follow_cart("FollowCartesianTarget", true),
      ac_follow_joint("FollowJointTarget", true), 
      ac_solveIK("SolveIK", true),
      T_B_Bcam_(Matrix3d::Identity(), Vector3d::Zero()), nh_(_node_handle),
      kill_this_node_(kill_this_node)
{
  if (!nh_.getParam("loop_rate", loop_rate_))
  {
    loop_rate_ = 50;
  }

  if (!nh_.getParam("unit_id", unit_id))
  {
    unit_id = 0;
  }

  if (!nh_.getParam("samples_number", samples_num_))
  {
    // samples_num_ = 5;
    samples_num_ = 10;
  }
  
  if (!nh_.getParam("waypoints_num", waypoints_num_))
  {
    waypoints_num_ = 5;
  }

  ompl_timeout_ = 3.0;
  // ToDO - get double values from the string input  
  // if (!nh_.getParam("arms_offset", arms_tf_offset))
  // {
  //   arms_tf_offset << 0.28, -0.6, 0.0;
  // }
  // iterServicing = 0;
  // Subscribers

  sub_asar_ee_pose_ = nh_.subscribe(
      "asar/ee", 1, &AsarHybridTmp::SubGetAsarEEPoseCb, this);



  service1 = nh_.advertiseService("check_pose_for_ik",
                                  &AsarHybridTmp::isPoseIKReachable, this);

  service2 = nh_.advertiseService("is_holding",
                                  &AsarHybridTmp::IsHolding, this);

  suture_ik_service_ = nh_.advertiseService("check_if_partner_can_reach",
                                  &AsarHybridTmp::checkIfPartnerCanGraspGivenExpectedNeedlePose, this);

  sample_pose_service = nh_.advertiseService("get_grasp_samples",
                                  &AsarHybridTmp::getSamples, this);
  
  get_obj_pose_service = nh_.advertiseService("get_poses_for_pddl",
                                  &AsarHybridTmp::getObjectPoseCb, this);
  motion_handler_service = nh_.advertiseService("motion_handler_pddl",
                                  &AsarHybridTmp::motions_service_cb, this);
  Matrix3d R_B_Fee_cmd_init;
  Vector3d pdes_init;

  // Subscribers
  sub_needle_pose_ =
      nh_.subscribe("/needle/poses", 1, &AsarHybridTmp::subNeedleCb, this);
  sub_needle_offs_ =
      nh_.subscribe("/needle/offsets", 1, &AsarHybridTmp::subOffsetsCb, this);

  sub_needle_tip_ =
      nh_.subscribe("/needle/tip", 1, &AsarHybridTmp::subTipPoseCb, this);
  sub_tissue_points =
      nh_.subscribe("/tissue_points", 1, &AsarHybridTmp::subTissPointsCb, this);

  pub_grasps_counter_ = nh_.advertise<std_msgs::Int32>("grasps_counter", 1);
  
  if (unit_id == 0)
  {
    // Safety workspace center
    // ws_center_ << 0.420, 0.305, 0.104; // old
    ws_center_ << 0.22, 0.44, 0.13; // new

    lx_ =  0.2;  //0.18; 0.12
    ly_ =  0.2;  //0.18; 0.12
    lz_ =  0.2;  //0.18; // 0.15

    pub_grip_trigger_ =
        nh_.advertise<std_msgs::String>("/unit0/asar/gripper/grip_cmd", 1);
    pub_needle_pose_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/needle_expected_pose", 1);

    sub_contact_info_ = nh_.subscribe(
        "/arm0/tip/contact", 1, &AsarHybridTmp::subSimContactInfo, this);
   
    handoverReachabilityClient = nh_.serviceClient<asar_hybrid_tmp::checkIfPartnerCanGrasp>(
      "/unit1/check_if_partner_can_reach");
  }
  else if (unit_id == 1)
  {

    // arms_tf_offset << 0.28, -0.6, 0.0; old
    arms_tf_offset << 0.0, -0.88, 0.0;

    // ws_center_ << 0.7, -0.325, 0.104; // old setup
    ws_center_ << 0.22, -0.44, 0.13; // new setup
    lx_ = 0.2; //0.14; 0.12
    ly_ = 0.2; //0.18; 0.12
    lz_ = 0.2; //0.20; //0.15
    pub_grip_trigger_ =
        nh_.advertise<std_msgs::String>("/unit1/asar/gripper/grip_cmd", 1);

    sub_contact_info_ = nh_.subscribe(
        "/arm1/tip/contact", 1, &AsarHybridTmp::subSimContactInfo, this);
    handoverReachabilityClient = nh_.serviceClient<asar_hybrid_tmp::checkIfPartnerCanGrasp>(
      "/unit0/check_if_partner_can_reach");
  }

  else
  {
    ROS_ERROR_STREAM("Unit ID is not valid");
  }

  // Publishers
  pub_T_B_Fee_cmd_ =
      nh_.advertise<geometry_msgs::PoseStamped>("asar/ee/cmd", 1);
  pub_asar_grip_cmd_ = nh_.advertise<std_msgs::Float32>("asar/gripper/cmd", 1);
  pub_expectedPose = nh_.advertise<geometry_msgs::Point>("expected_pose", 1);
  pub_actualPose = nh_.advertise<geometry_msgs::Point>("actual_pose", 1);

  omplClient = nh_.serviceClient<asar_hybrid_tmp::getPath>(
      "/ompl_planner_service");

  collisionCheckerClient = nh_.serviceClient<asar_hybrid_tmp::collisionCheck>(
      "/check_collisions_fcl");
  
  suturePathClient = nh_.serviceClient<asar_hybrid_tmp::SuturePath>(
      "/suture_path_service");
  suturePointsClient = nh_.serviceClient<asar_hybrid_tmp::SuturePoints>(
      "/suture_service");
  
  addNoiseClient_ = nh_.serviceClient<asar_hybrid_tmp::AddNoise>(
      "/add_noise_service");

  pub_asar_grip_cmd_msg_.data = 0.0;

  last_time_ = ros::Time::now().toSec();
  cmd_time_ = ros::Time::now().toSec();
  ROS_INFO_STREAM("cmd_time: " << cmd_time_);
  ROS_INFO_STREAM("last_time: " << last_time_);

  iface_grip_angle_ = 0.02;
  mapping_done_ = false;

  // Safety workspace
  S_ul_.push_back(ws_center_[0] + lx_ / 2);
  S_ul_.push_back(ws_center_[1] + ly_ / 2);
  S_ul_.push_back(ws_center_[2] + lz_ / 2);
  S_ll_.push_back(ws_center_[0] - lx_ / 2);
  S_ll_.push_back(ws_center_[1] - ly_ / 2);
  S_ll_.push_back(ws_center_[2] - lz_ / 2);

  contactMade = false;
  safetyDistance = 0.002;
  

  // Action Server
  graspActionServ_ = std::make_shared<
      actionlib::SimpleActionServer<asar_hybrid_tmp::GraspAction>>(
      nh_, "arm_grasp_server",
      boost::bind(&AsarHybridTmp::executeCB, this, _1), false);
  graspActionServ_->start();
  pathFollowerServ_ = std::make_shared<
      actionlib::SimpleActionServer<asar_hybrid_tmp::PathFollowerAction>>(
      nh_, "path_follower_server",
      boost::bind(&AsarHybridTmp::pathFollowerCB, this, _1), false);
  pathFollowerServ_->start();
  // ROS_WARN("constructer ok ..?");
  string urdf_path = "/home/nir/asar_ws/src/asar_description/urdf/asar_v2_ik_solver.urdf"; 
  pinocchio::urdf::buildModel(urdf_path,  model);
  
  lastPose = T_B_Fee_;
  // std::cout << unit_id << ": lastPose" << lastPose.translation();
  ROS_INFO("grasp server initialized");
}

AsarHybridTmp ::~AsarHybridTmp()
{

}

void AsarHybridTmp::subNeedleCb(
    const geometry_msgs::PoseArray::ConstPtr &msg)
{
  poseMsgToSE3(msg->poses[2], needle_wayp);

  poseMsgToSE3(msg->poses[0], w1);
  poseMsgToSE3(msg->poses[1], w2);
  poseMsgToSE3(msg->poses[2], w3);
  poseMsgToSE3(msg->poses[3], mid);
  needleWaypoints.poses = msg->poses;
  if (unit_id == 1)
  {
    mid.translation() = mid.translation() + arms_tf_offset;
  }
}

void AsarHybridTmp::subTissPointsCb(
    const geometry_msgs::PoseArray::ConstPtr &msg)
{
  tissuePoints.poses = msg->poses;
  // ROS_INFO("tissue points cb");
}

void AsarHybridTmp::subOffsetsCb(
    const geometry_msgs::PoseArray::ConstPtr &msg)
{

  // poseMsgToSE3(msg->poses[0], offs0);
  poseMsgToSE3(msg->poses[0], offs1);
  poseMsgToSE3(msg->poses[1], offs5);
  poseMsgToSE3(msg->poses[2], offs4);
  poseMsgToSE3(msg->poses[3], tipOffset);
  poseMsgToSE3(msg->poses[4], tipReverseOffset);
}

void AsarHybridTmp::changeZ(float z)
{
  // ROS_WARN("change z called. ..");

  int       j = 0;
  ros::Rate r(20);

  eePose2 = T_B_Fee_;
  eePose2.translation().z() += z;
  while (j < 10)
  {

    limitWokspace(T_B_Fee_);
    sendSolverAndExecute(T_B_Fee_);
    T_B_Fee_cmd_ = T_B_Fee_;

    publishAsarEEPoseTarget();
    r.sleep();
    j++;
    // ROS_INFO_STREAM("changing z ..." << j);
  }

  while (j < 40)
  {

    limitWokspace(eePose2);
    sendSolverAndExecute(eePose2);
    T_B_Fee_cmd_ = eePose2;

    publishAsarEEPoseTarget();
    r.sleep();
    j++;
    // ROS_INFO_STREAM("changing z ..." << j);
  }
}

void AsarHybridTmp::moveToPose(pin::SE3 currentPose, pin::SE3 targetPose)
{
  // ROS_WARN("Move pose called ...");
  // ros::Rate r(loop_rate_);
  limitWokspace(targetPose);
  sendSolverAndExecute(targetPose);
  // r.sleep();
  T_B_Fee_cmd_ = targetPose;
  publishAsarEEPoseTarget();
  lastPose = targetPose;
  // ROS_WARN("Move pose exit ...");
}

void AsarHybridTmp::SubGetAsarEEPoseCb(
    const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  cmd_time_ = msg->header.stamp.now().toSec();
  poseMsgToSE3(msg->pose, T_B_Fee_);
  if (mapping_done_ == false)
    mapping_done_ = true;

  eefPose = msg->pose;
}

void AsarHybridTmp::subTipPoseCb(
    const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  poseMsgToSE3(msg->pose, tipPose);
  
  if(unit_id == 1)
    tipPose.translation() = tipPose.translation() + arms_tf_offset;
  if (storeTipPose)
    tipPoseTraj.push_back(tipPose);
}

void AsarHybridTmp::subSimContactInfo(
    const std_msgs::String::ConstPtr &msg)
{
  // ROS_INFO_STREAM("msg received ..." << msg );
  contactMade = true;
}
void AsarHybridTmp::publishAsarEEPoseTarget()
{
  // Publishing the target pose
  pub_T_B_Fee_cmd_msg_.header.stamp = ros::Time::now();
  poseSE3ToMsg(T_B_Fee_cmd_, pub_T_B_Fee_cmd_msg_.pose);
  pub_T_B_Fee_cmd_.publish(pub_T_B_Fee_cmd_msg_);
  // Publishing grip angle
  if (open_gripper)
    iface_grip_angle_ = 0.5;
  else
    iface_grip_angle_ = 0.1;
  pub_asar_grip_cmd_msg_.data = iface_grip_angle_;
  pub_asar_grip_cmd_.publish(pub_asar_grip_cmd_msg_);
  // ROS_WARN_STREAM(
  //     " pub_T_B_Fee_cmd_msg_.data = " << pub_asar_grip_cmd_msg_.data);
}

void AsarHybridTmp::limitWokspace(pin::SE3 &Xd)
{
  Vector3d pd = Xd.translation();
  for (int axis = 0; axis < 3; axis++)
  {
    if (pd[axis] > S_ul_[axis])
      pd[axis] = S_ul_[axis];
    else if (pd[axis] < S_ll_[axis])
      pd[axis] = S_ll_[axis];
  }
  Xd.translation() = pd;
}



void AsarHybridTmp::sendAsarJointTarget(VectorXd q_target)
{
  asar_control::FollowJointTargetGoal goal;
  goal.joint_target.position.resize(q_target.size());

  VectorXd::Map(&goal.joint_target.position[0], q_target.size()) = q_target;
  // ROS_INFO_STREAM("Waiting for action server to start");
  ac_follow_joint.waitForServer();

  ac_follow_joint.sendGoal(
      goal,
      boost::bind(&AsarHybridTmp::actFollowJointTargetDoneCb, this, _1,
                  _2),
      FollowJointTargetClient::SimpleActiveCallback(),
      FollowJointTargetClient::SimpleFeedbackCallback());

  ac_follow_joint.waitForResult(ros::Duration(5.0));

  // ROS_INFO_STREAM("Tracking Action server started. Goal sent");
}

void AsarHybridTmp::sendSolverTarget(pin::SE3 Xd)
{
  execute = false;
  asar_control::SolveIKGoal goal;

  poseSE3ToMsg(Xd, goal.target_pose);
  // ROS_INFO_STREAM("Target p:" << Xd.translation().transpose());
  // ROS_INFO_STREAM("Target R:" << Xd.rotation());

  // ROS_INFO_STREAM("Waiting for action server to start");
  ac_solveIK.waitForServer();

  ac_solveIK.sendGoal(
      goal, boost::bind(&AsarHybridTmp::actSolveIKDoneCb, this, _1, _2),
      SolveIKClient::SimpleActiveCallback(),
      SolveIKClient::SimpleFeedbackCallback());
  // ROS_INFO_STREAM("Planning Action server started. Goal sent");
  ac_solveIK.waitForResult();
}

void AsarHybridTmp::sendSolverAndExecute(pin::SE3 Xd)
{
  execute = true;
  asar_control::SolveIKGoal goal;

  poseSE3ToMsg(Xd, goal.target_pose);
  // ROS_INFO_STREAM("Target p:" << Xd.translation().transpose());
  // ROS_INFO_STREAM("Target R:" << Xd.rotation());

  // ROS_INFO_STREAM("Waiting for action server to start");
  ac_solveIK.waitForServer();

  ac_solveIK.sendGoal(
      goal, boost::bind(&AsarHybridTmp::actSolveIKDoneCb, this, _1, _2),
      SolveIKClient::SimpleActiveCallback(),
      SolveIKClient::SimpleFeedbackCallback());
  ac_solveIK.waitForResult();
  // ROS_INFO_STREAM("Planning Action server started. Goal sent");
  execute = false;
}



void AsarHybridTmp::actFollowJointTargetDoneCb(
    const actionlib::SimpleClientGoalState &             state,
    const asar_control::FollowJointTargetResultConstPtr &result)
{
  bool res = result->succeed;
  // if (res == true)
  // {
  //   ROS_INFO_STREAM("Joint target reached");
  // }
  // else
  // {
  //   ROS_WARN_STREAM("Joint target aborted ");
  // }
}

void AsarHybridTmp::actSolveIKDoneCb(
    const actionlib::SimpleClientGoalState &   state,
    const asar_control::SolveIKResultConstPtr &result)
{
  ikSolved = false;
  bool res = result->succeed;
  if (res == true)
  {
    VectorXd q_sol;
    q_sol = VectorXd::Map(&result->joint_sol.position[0],
                          result->joint_sol.position.size());
    // ROS_INFO_STREAM(
    //     "Solution found. Sending Joint Cmd to ASAR: " << q_sol.transpose());
    q_solved = q_sol;
    if (execute)
    {
      sendAsarJointTarget(q_sol);
      // ROS_WARN("Joint Target Sent ...");
    }

    ikSolved = true;
  }
  // else
  // {
  //   ROS_WARN_STREAM("Solution not found");
  //   // sleep(1.0);
  // }
}

pin::SE3 AsarHybridTmp::applyRotationToPose(const pin::SE3& needlePose, double xAngle,
                                          double yAngle, double zAngle)
{


  Eigen::AngleAxisd yawAngle(zAngle, Eigen::Vector3d::UnitZ());


  Eigen::AngleAxisd rollAngle(xAngle, Eigen::Vector3d::UnitX());

  Eigen::AngleAxisd pitchAngle(yAngle, Eigen::Vector3d::UnitY());

  Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
  q.normalize();
  Eigen::Matrix3d rotationMatrix = q.matrix();

  pin::SE3 rotatedPose;
  rotatedPose.rotation() = needlePose.rotation() * rotationMatrix;
  rotatedPose.translation() = needlePose.translation();
  return rotatedPose;
}

void AsarHybridTmp::controlLoop()
{
  
  ros::Rate loop_rate(1);
  while (not(*kill_this_node_))
  {
    // T_B_Fee_cmd_ = mid;
    // publishAsarEEPoseTarget();
    loop_rate.sleep();
    std_msgs::Int32 msg;

    //for pddl only  use the counter
    // msg.data = counter;
    msg.data = grasps_counter;
    pub_grasps_counter_.publish(msg);
  }   
}

bool AsarHybridTmp::isPoseIKReachable(
    asar_hybrid_tmp::isPoseReachable::Request & req,
    asar_hybrid_tmp::isPoseReachable::Response &res)
{
  // ROS_WARN(" check pose service called ...");
  // ROS_INFO_STREAM(" request pose = " << req);
  // ros::Rate r(1);

  pin::SE3 sendToIKPose;
  poseMsgToSE3(req.checkPose, sendToIKPose);
  limitWokspace(sendToIKPose);
  sendSolverTarget(sendToIKPose);

  geometry_msgs::Pose p;
  p = req.checkPose;

  execute = false;
  if (ikSolved)
  {
    // ROS_WARN("solved ..");
    res.valid = true;
    for (size_t i = 0, size = q_solved.size(); i < size; i++) { 
      res.jointValues.push_back(q_solved(i));
    }
    return true;
    // ROS_INFO_STREAM("res = " << res);
  }
  
  // ROS_WARN("not solved");
  return true;

  
}


void AsarHybridTmp::pathFollowerCB(const asar_hybrid_tmp::PathFollowerGoalConstPtr &goal)
{
  ros::Rate r(loop_rate_);
  asar_hybrid_tmp::PathFollowerResult res;
  
  if (goal->command == "move_holding")
  {
    // assumes that already holding the needle ...
    //supposedly should just execute the trajectory?
    ROS_INFO("move holding path received");
    pin::SE3 waypoint;
    for (auto p : goal->path)
    {
      poseMsgToSE3(p, waypoint);
      moveToPose(T_B_Fee_, waypoint);
      r.sleep();
      lastPose = waypoint;
    }
    res.success = true;
    pathFollowerServ_->setSucceeded(res);
    return;

  }  

  if (goal->command == "grasp_n")
  {
    ROS_INFO("grasp_n received");
    // sould move to pose and just grasp in the end? ...
    pin::SE3 waypoint;
    for (auto p : goal->path)
    {
      
      poseMsgToSE3(p, waypoint);
      moveToPose(T_B_Fee_, waypoint);
      lastPose = waypoint;
      r.sleep();
    }
    graspControl("grasp", waypoint);
    r.sleep();
    
    res.success = true;
    pathFollowerServ_->setSucceeded(res);
    return;

  }  
  
  if (goal->command == "insert_n" )
  {
    // ROS_WARN_STREAM("path follower cmd = "<< goal->command);
    
    vector<GraspWaypoints> combined_path;
    for (auto gt : goal->combined_path)
    {
      GraspWaypoints p;
      poseMsgToSE3(gt.grasp, p.grasp);
      for (auto w : gt.trajectory)
      {
        pin::SE3 wp;
        poseMsgToSE3(w, wp);
        p.waypoints.push_back(wp);
      }

      p.grasp = p.waypoints[0];
      combined_path.push_back(p);
    }
    counter = combined_path.size();
    // ROS_WARN_STREAM("inser_n combined_path size = " << combined_path.size());
    // ROS_WARN_STREAM("inser_n counter = " << counter);
    executeGraspTrajectoryList(combined_path);
    // graspControl("open", lastPose);
    ROS_INFO("insert_n follow completed ...");
    // sleep(2.0);
    res.success = true;
    pathFollowerServ_->setSucceeded(res);

    return;
  }

  if (goal->command == "extract_n" )
  {
    
    vector<GraspWaypoints> combined_path;
    for (auto gt : goal->combined_path)
    {
      GraspWaypoints p;
      poseMsgToSE3(gt.grasp, p.grasp);
      for (auto w : gt.trajectory)
      {
        pin::SE3 wp;
        poseMsgToSE3(w, wp);
        p.waypoints.push_back(wp);
      }

      p.grasp = p.waypoints[0];
      combined_path.push_back(p);
    }
    counter += combined_path.size();
    // ROS_WARN_STREAM("extract_n combined_path size = " << combined_path.size());
    // ROS_WARN_STREAM("extract_n counter = " << counter);
    executeGraspTrajectoryList(combined_path);
    // graspControl("open", lastPose);
    ROS_INFO("extract_n follow completed ...");
    
    // sleep(2.0);
    res.success = true;
    pathFollowerServ_->setSucceeded(res);
    return;
  }

  ROS_INFO("unspecied goal type ...");
  pathFollowerServ_->setPreempted();
}

void AsarHybridTmp::executeCB(
    const asar_hybrid_tmp::GraspGoalConstPtr &goal)
{
  asar_hybrid_tmp::GraspResult res;
  open_gripper = false;
  ros::Rate loop_rate(loop_rate_);

  pin::SE3            targetForArm;
  boost::mutex::scoped_lock lockAct(mtx_Act_track_);

  m_curAct_track_ = ACT_EXECUTE;
  lockAct.unlock();
  Vector3d p_in, p_out;
  float cost;
  if(m_curAct_track_ == ACT_EXECUTE)
  {
    std::vector<PoseAndJointValues> samples, sortedSamples;

    if (goal->command == "store")
    {
      ROS_INFO("storing current pose ...");
      // ROS_INFO_STREAM("unit " << unit_id);
      initialPose = T_B_Fee_;
      // ROS_INFO_STREAM("init pose = " << initialPose);
      if (unit_id == 0)
      initialPose.translation().z() += 0.01;
      if (unit_id == 1)
      {
        // initialPose.translation().x() = 0.24;
        initialPose.translation().y() += 0.04;
        // initialPose.translation().z() = 0.15;
        T_B_Fee_cmd_ = T_B_Fee_;
        publishAsarEEPoseTarget();
        loop_rate.sleep();
        sleep(1);
        T_B_Fee_cmd_ = initialPose;
        publishAsarEEPoseTarget();
        loop_rate.sleep();
        sleep(1);
      }      

      loop_rate.sleep();
      res.success = true;
      graspActionServ_->setSucceeded(res);
      return;
    }

    if (goal->command == "reset")
    {
      graspControl("open", T_B_Fee_);
      ROS_INFO("new reset arm position is called  ...");
      graspControl("close_gripper", T_B_Fee_);
      goToPosition(initialPose);
      loop_rate.sleep();
      lastPose = T_B_Fee_;
      loop_rate.sleep();
      grasps_counter = 0;
      counter = 0;
      res.success = true;
      graspActionServ_->setSucceeded(res);
      return;
    }
    
    if (goal->command == "grasp_n")
    {
      graspControl("open", T_B_Fee_);
      pin::SE3 graspPose;
      poseMsgToSE3(goal->grasp, graspPose);
      if (graspPose.translation().norm() == 0)
      {
        ROS_ERROR("resampling new grasp, not given as input ...");
        res.success = sendGraspNeedle("insert", unit_id);
        add_noise_to_grasp();
        graspActionServ_->setSucceeded(res);
        return;
      }
      auto traj = getPathWithOmpl(lastPose, graspPose, unit_id, "grasp", cost);
      if (traj.size() == 0)
      {
        res.success = false;
        graspActionServ_->setSucceeded(res);
        return; 
      }
      for (auto tp : traj)
      {
        moveToPose(T_B_Fee_, tp);
        loop_rate.sleep();
        lastPose = tp;
      }

      loop_rate.sleep();
      graspControl("grasp", lastPose);      
      res.success = true;
      graspActionServ_->setSucceeded(res);
      return;
    }
    
    if (goal->command == "move_holding")
    {
      ROS_INFO("inside move holding ...");
      res.success = true;
      pin::SE3 goToPose;

      float angle;

      getInsertionPointsAndAngleFromId("insert", goal->id, p_in, p_out, angle);

      vector<pin::SE3> insert_points;
      insert_points = generate_waypoints("insert", p_in, p_out, angle, waypoints_num_);
      // ROS_INFO_STREAM("move_holding requested id = " << goal->id);
      bool reached = false;
      goToPose = insert_points[0];
      bool regrasped = false;
      reached = moveHolding(goToPose, unit_id, "insert", p_in, p_out, regrasped);
      res.success = reached;

      if(!reached)
      {
        graspControl("open", T_B_Fee_);
        ROS_WARN("move_holding not reached reset arm position ...");
        moveToPose(T_B_Fee_, initialPose);
        loop_rate.sleep();
        holding = "";
      }
      loop_rate.sleep();
      graspActionServ_->setSucceeded(res);
      return;
    }

    if (goal->command == "insert_n")
    {
      float angle;

      getInsertionPointsAndAngleFromId("insert", goal->id, p_in, p_out, angle);

      vector<pin::SE3> insert_points;
      insert_points = generate_waypoints("insert", p_in, p_out, angle, waypoints_num_);

      int regrasps = 0;
      res.success = followWaypoints(insert_points, "insert", p_in, p_out, regrasps );
      tipPoseWaypoints.clear();
      tipPoseTraj.clear();
      loop_rate.sleep();
      if (res.success)
      {
        asar_hybrid_tmp::SuturePoints points_srv;
        points_srv.request.id = goal->id;
        points_srv.request.type = "inserted";
        suturePointsClient.call(points_srv);
   
      }
      graspControl("open", T_B_Fee_);
      ROS_WARN("insert_n completed ..."); 
      holding = "";
      graspActionServ_->setSucceeded(res);
      return;
    }
    
    if (goal->command == "extract_n")
    {
      ROS_ERROR("extract n is requested ...");
      float angle;

      getInsertionPointsAndAngleFromId("insert", goal->id, p_in, p_out, angle);
      vector<pin::SE3> extract_points;
      extract_points = generate_waypoints("extract", p_in, p_out, angle, waypoints_num_);

      int regrasps = 0;
      res.success = followWaypoints(extract_points, "extract", p_in, p_out, regrasps );
      tipPoseWaypoints.clear();
      tipPoseTraj.clear();
      if (res.success)
      {
        asar_hybrid_tmp::SuturePoints points_srv;
        points_srv.request.id = goal->id;
        points_srv.request.type = "extracted";
        suturePointsClient.call(points_srv);

        if (unit_id == 1)
          holding = "unit1";
        else
          holding = "unit0";
      }
      else
      {
        graspControl("open", T_B_Fee_);
        goToPosition(initialPose);
        lastPose = initialPose;
        loop_rate.sleep();
        graspControl("close_gripper", initialPose);
        loop_rate.sleep();
      }
      loop_rate.sleep();
      graspActionServ_->setSucceeded(res);
      return;
    }

    if (goal->command == "release")
    {
      graspControl("open", T_B_Fee_cmd_);
      loop_rate.sleep();
      targetForArm =  T_B_Fee_cmd_;
      applyOffset(targetForArm, 0.005);
      goToPoseWithOmpl(targetForArm, unit_id, "move");
      graspControl("close_gripper", targetForArm);
      goToPoseWithOmpl(initialPose, unit_id, "move");

      res.success = true;
     
      graspActionServ_->setSucceeded(res);
      return;
    }

    if (goal->command == "coll_check_viz")
    {
      graspControl("close_gripper", targetForArm);
      goToPoseWithOmpl(T_B_Fee_cmd_, unit_id, "move");
      graspControl("close_gripper", targetForArm);
      res.success = true;
      graspActionServ_->setSucceeded(res);
      return;
    }

    if (goal->command == "pick")
    {
      sendGraspNeedle("insert", unit_id);
      targetForArm =  T_B_Fee_cmd_;
      applyOffset(targetForArm, 0.01);
      goToPoseWithOmpl(targetForArm, unit_id, "move");  
      res.success = true;
      graspActionServ_->setSucceeded(res);
      return;
    }

    if (goal->command == "grasp")
    {

      // grasp_needle for sequential case only? 
      // if (unit_id == 1)
      //     sendGraspNeedle(string("extract"), unit_id);
      // else    
      //     sendGraspNeedle(string("insert"), unit_id);
      res.success = sendGraspNeedle("extract", unit_id);
      add_noise_to_grasp();
      loop_rate.sleep();
      graspActionServ_->setSucceeded(res);
      
      return;

    }  


    if (goal->command == "store_suture_trajectory")
    {
      lastPose = T_B_Fee_;
      float angle;
      
      getInsertionPointsAndAngleFromId("insert", goal->id, p_in, p_out, angle);

      vector<pin::SE3> insert_points, extract_points;
      insert_points = generate_waypoints("insert", p_in, p_out, angle, goal->num_waypoints);
      extract_points = generate_waypoints("extract", p_in, p_out, angle, goal->num_waypoints);
      samples_num_ = goal->num_samples;
      waypoints_num_ = goal->num_waypoints;
      ompl_timeout_ = goal->ompl_timeout;
      
      bool regrasped = false;
      // sendGraspNeedle(string("insert"), unit_id);
      moveHolding(insert_points[0], unit_id, "insert", p_in, p_out, regrasped);
      pin::SE3 grasp = T_B_Fee_;
      int regrasps = 0;
      storeTipPose = true;
      res.success = followWaypoints(insert_points, "insert", p_in, p_out, regrasps);
      writePointsToFile("random",insert_points, tipPoseTraj, "insert");
      tipPoseWaypoints.clear();
      tipPoseTraj.clear();
      // storeTipPose = false;

      ROS_WARN("insertion completed. ..");
      // ROS_INFO_STREAM("regrasps = " << regrasps);
      res.regrasps_num += regrasps;
      regrasped = false;
      moveHolding(extract_points[0], unit_id, "extract", p_in, p_out, regrasped);
      res.success = followWaypoints(extract_points, "extract", p_in, p_out, regrasps );
      writePointsToFile("random",extract_points, tipPoseTraj, "extract");
      tipPoseWaypoints.clear();
      tipPoseTraj.clear();
      storeTipPose = false;
      ROS_WARN("extraction completed. ..");
      // ROS_INFO_STREAM("regrasps = " << regrasps);
      res.regrasps_num += regrasps;
      // ROS_INFO_STREAM("total res.regrasps_num = " << res.regrasps_num);
      graspActionServ_->setSucceeded(res);  
      ROS_INFO("store trajectory completed...");    
      return;

    }  

    if (goal->command == "sampling_test")
    {
      float angle;

      getInsertionPointsAndAngleFromId("insert", goal->id, p_in, p_out, angle);
      
      vector<pin::SE3> insert_points, extract_points;

      insert_points = generate_waypoints("insert", p_in, p_out, angle, waypoints_num_);
      extract_points = generate_waypoints("extract", p_in, p_out, angle, waypoints_num_);

      bool regrasped = false;
      moveHolding(insert_points[0], unit_id, "insert", p_in, p_out, regrasped);
      pin::SE3 grasp = T_B_Fee_;
      int regrasps = 0;
      res.success = followWaypoints(insert_points, "insert", p_in, p_out, regrasps);
      tipPoseWaypoints.clear();
      tipPoseTraj.clear();
      // storeTipPose = false;
      if (!res.success)
        graspActionServ_->setPreempted();
      ROS_WARN("insertion completed. ..");
      
      res.regrasps_num += regrasps;
      regrasped = false;
      moveHolding(extract_points[0], unit_id, "extract", p_in, p_out, regrasped);
      res.success = followWaypoints(extract_points, "extract", p_in, p_out, regrasps );
      tipPoseWaypoints.clear();
      tipPoseTraj.clear();

      if (!res.success)
        graspActionServ_->setPreempted();
      ROS_WARN("extraction completed. ..");
      graspActionServ_->setSucceeded(res);      
      return;

    }  

    if (goal->command == "handover")
    {
      // basically move to the handover region
      // res.success = randomizedHandoverRegionSampler(100);
      res.success = handoverRegionSampler();
      loop_rate.sleep();
      if (res.success)
        ROS_INFO("handover success!");
      else
        ROS_WARN("didnt reach handover region ...");
      graspActionServ_->setSucceeded(res);  
      return;
    }    
    publishAsarEEPoseTarget();
    loop_rate.sleep();

    m_curAct_track_ = ACT_NONE;
    mtx_Act_track_.unlock();
  }
  
  
}

bool AsarHybridTmp::canReachToTipPose(pin::SE3 startNeedleP, pin::SE3 goalNeedleP, pin::SE3 offset, vector<pin::SE3>& traj)
{
  float cost;
  pin::SE3 goalEefPose, startEffpose ;

  startEffpose = startNeedleP * offset.inverse();
  goalEefPose = goalNeedleP * offset.inverse();

  traj = getPathWithOmpl(startEffpose, goalEefPose, unit_id, "grasp", cost);

  if (traj.size() > 0)
      return true;
  // ROS_INFO("cant reach a waypoint ...");
  return false;
}

bool AsarHybridTmp::isGraspValidForExpectedPose(PoseAndJointValues& currentGrasp, pin::SE3& goalPose, string type, Vector3d p_in, Vector3d p_out)
    {
    pin::SE3 ofst = currentGrasp.offset;
    pin::SE3 targetGrasp = goalPose*ofst.inverse();

    Vector3d testingPoint, diff;
    if (isPoseKinematicallyValid(targetGrasp))
    {
        // ROS_INFO("is pose kin valid true ...");
        // std::cout << "diff = " << diff.norm() << std::endl;
        if (type == "insert")
        {
          // ROS_INFO("insert check ...");
          testingPoint = targetGrasp.translation();
          diff = testingPoint - p_in;

          return (testingPoint(2) - p_in(2) ) > 0.002;
          // return testingPoint(2) > p_in(2);
          return true;
        }

        else if (type == "extract")
        {
          // ROS_INFO("exract check ...");
          testingPoint = targetGrasp.translation();
          diff = testingPoint - p_out;
          // return diff.norm() > safetyDistance;
          return (testingPoint(2) - p_out(2) ) > 0.002;
          // return testingPoint(2) > p_out(2);
          return true;
        }
        // ROS_INFO("else condition .. is pose kin valid false ...");
        return true;
    }
    
    // bool check2 = isGraspValid(currentGrasp.pose, goalPose, type, p_in, p_out);
    // cout << "check2 ~= " << check2 << endl;
    // sleep(1.0);
    return false;
    }


void AsarHybridTmp::samplePosesFromExpectedNeedlePose(pin::SE3 expectedTipPose, string type, double initAngle, int samplesNum, vector<PoseAndJointValues>& reachablePoses)
{
    // ROS_INFO("Sampling poses from expected tipPose...");

    pin::SE3 sample_offset;
    double sampleAngle = 0;

    if (type == "insert")
    {
        sample_offset = offs5;
        initAngle = -M_PI / 8;
    }
    else if (type == "extract")
    {
        sample_offset = offs4;
        initAngle = -M_PI / 6;
    }
    else
    {
        // ROS_INFO("Sampling else expected tip pose...");
        sample_offset = offs1;
        initAngle = -M_PI / 10;
    }

    std::vector<int> z_axis_angles, x_axis_angles;

    z_axis_angles = generateRandomUniqueIntegers(samplesNum, 0, 45);
    x_axis_angles = generateRandomUniqueIntegers(samplesNum, -30, 30);

    reachablePoses.clear();
    reachablePoses.reserve(samplesNum * samplesNum * 2);

    PoseAndJointValues sample;
    pin::SE3 expectedMid = expectedTipPose * calculateOffset(mid, tipPose).inverse();

    #pragma omp parallel for collapse(2)
    for (int z_i_ : z_axis_angles)
    {
        for (int x_i_ : x_axis_angles)
        {
            double zangle = (M_PI / 180) * z_i_;
            double xangle = (M_PI / 180) * x_i_;

            sampleAngle = initAngle + (M_PI / 180) * z_i_;
            pin::SE3 sample_pose = applyRotationToPose(expectedMid, 0, 0, sampleAngle);
            sample_pose = sample_pose * sample_offset;
            sample_pose = applyRotationToPose(sample_pose, xangle, 0, 0);
            sample_pose = applyRotationToPose(sample_pose, 0, zangle, 0);
            

            if (isPoseKinematicallyValid(sample_pose) )
            {
                sample.pose = sample_pose;
                sample.jointValues = q_solved;
                sample.offset = calculateOffset(sample_pose, expectedTipPose);

                if (checkIfPoseCollisionFree(sample))
                {
                    #pragma omp critical
                    {
                        reachablePoses.push_back(sample);
                    }
                }
            }

            sample_pose = applyRotationToPose(sample_pose, 0, M_PI, 0);

            if (isPoseKinematicallyValid(sample_pose) )
            {
                sample.pose = sample_pose;
                sample.jointValues = q_solved;
                sample.offset = calculateOffset(sample_pose, expectedTipPose);

                if (checkIfPoseCollisionFree(sample))
                {
                    #pragma omp critical
                    {
                        reachablePoses.push_back(sample);
                    }
                }
            }
        }
    }

    // ROS_INFO_STREAM("Poses size = " << reachablePoses.size());
}



AsarHybridTmp::PoseAndJointValues AsarHybridTmp::updateUsingExpectedGraspPose(pin::SE3 &goalPose, pin::SE3 &expectedTip, 
        bool &updated, string type, Vector3d p_in, Vector3d p_out)
    {
    
    ros::Rate r(loop_rate_);
    PoseAndJointValues newGrasp;

    vector<PoseAndJointValues> new_samples, new_sorterd_samples;

    samplePosesFromExpectedNeedlePose(expectedTip, type,M_PI/8, samples_num_, new_samples);
    sortPosesByHighestManipulabilityScore(new_samples, new_sorterd_samples);
    updated=false;
    for (PoseAndJointValues p : new_sorterd_samples)
    {
        // T_B_Fee_cmd_ = p.pose;
        // publishAsarEEPoseTarget();
        if (isGraspValidForExpectedPose(p, goalPose, type, p_in, p_out) )
        {
            // T_B_Fee_cmd_ = p.pose;
            // publishAsarEEPoseTarget();
            // ROS_INFO_STREAM("found a valid update type = " << type);
            newGrasp = p;
            updated = true;
            break;
        }

        r.sleep();
    }

    return newGrasp;

    }

bool AsarHybridTmp::getGraspTrajectoryList(pin::SE3 startGrasp, pin::SE3 startOffset, pin::SE3 expectedTipPose, vector<pin::SE3> waypoints,
         string type, Vector3d p_in, Vector3d p_out, int &regrasp_num, vector<GraspWaypoints> &combined_path)
  {
    
    // ROS_WARN_STREAM("get grasp traj called type = " <<  type);
    // ROS_WARN_STREAM("waypoints size  = " <<  waypoints.size());
    vector<tuple<pin::SE3, vector<pin::SE3>>> my_vec;
    vector<GraspWaypoints> gws;
    vector<pin::SE3> grasps_temp;

    bool has_update = false;

    PoseAndJointValues grasp;
    grasp.pose = startGrasp;
    grasp.offset = calculateOffset(grasp.pose, tipPose);
    // if (type =="extract")
    //   grasp.offset = startOffset;


    vector<pin::SE3> empty_arr;

    ros::Rate r(loop_rate_);
    vector<MultiGraspTraj> suturePath;
    MultiGraspTraj sPath;
    sPath.id = 0;
    for (int i = 0; i < waypoints.size(); i++)
    {
      vector<pin::SE3> traj;

      // first check current grasp
      bool grasp_valid = false;
      pin::SE3 tipExpectedP;
      if (i == 0)
        tipExpectedP = expectedTipPose;
      

      bool hasTraj = true; 
      if (isGraspValidForExpectedPose(grasp, waypoints[i], type, p_in, p_out))
      {
        // if it is valid try to get the trajectory
        hasTraj = canReachToTipPose(tipExpectedP, waypoints[i], grasp.offset, traj);
        if (!hasTraj)
        {
          // cout << " i  = " << i << endl;
          // ROS_ERROR("inside valid grasp   no traj ...");
          // continue;
          return false;
        }
        
        // ROS_INFO_STREAM("traj size = " <<  traj.size());
        grasp.pose = traj.back();
        grasp.offset = calculateOffset(grasp.pose, waypoints[i]);
        sPath.grasp = grasp.pose;
        sPath.traj = traj;
      }

      // second need to update the grasp w.r.t to expectedTipPose
      else
      {
        // ROS_WARN("grasp not valid ...");
        grasp = updateUsingExpectedGraspPose(waypoints[i],tipExpectedP, has_update, type, p_in, p_out);
        
        if (!has_update)
        {
          // ROS_WARN("no updated grasp !");
          return false;
        }
        grasp.offset= calculateOffset(grasp.pose, tipExpectedP);
        hasTraj = canReachToTipPose(tipExpectedP, waypoints[i], grasp.offset, traj);
        if (!hasTraj)
        {

          return false;
        }

        
        // ROS_INFO_STREAM("updated grasp traj size = " <<  traj.size());

        regrasp_num++;
        sPath.id = regrasp_num;
        sPath.grasp = grasp.pose;
        sPath.traj = traj;
      }
      // ROS_INFO_STREAM("traj arr size = " << traj.size());
      // sleep(1.0);
      // my_vec.push_back(make_tuple(grasp.pose, traj));
      // grasps_temp.push_back(grasp.pose);
      tipExpectedP = waypoints[i];
      r.sleep();
      suturePath.push_back(sPath);
      // if (type =="insert")
      //   ROS_INFO_STREAM("insert regrasps=   " << regrasp_num);
    }
    

    
    gws.clear();
    vector<MultiGraspTraj> combinedArr;
    combinedArr = combineTrajectories(suturePath);
    for  (auto& s : combinedArr)
    {
      GraspWaypoints g;
      g.grasp = s.grasp;
      g.waypoints = s.traj;
      gws.push_back(g);
    }
    
    // ROS_WARN_STREAM("gws size = " << gws.size());
    // ROS_WARN_STREAM("update_count = " << update_count);
    // sleep(3.0);
    combined_path = gws;
    // executeGraspTrajectoryList(gws);
    return true;
  }
  
bool AsarHybridTmp::executeGraspTrajectoryList(vector<GraspWaypoints> grasps)
  {

    ros::Rate r(loop_rate_);
    // pin::SE3 l = T_B_Fee_;
    if (lastPose.translation().norm() == 0)
        lastPose = T_B_Fee_;
    r.sleep();
    for (int j=0; j<grasps.size(); j++)
    {
      r.sleep();

        graspControl("open", lastPose);
        r.sleep();

        pin::SE3 new_g = grasps[j].grasp;
        // applyOffset(new_g, 0.005);
        new_g = normalizeOrientation(new_g);

        // try adding teleport here? without planning for regrasp trajectory ...
        moveToPose(lastPose, new_g);

        r.sleep();
        graspControl("grasp", new_g);
        r.sleep();
        
      // }

      for (auto p : grasps[j].waypoints)
      {
        // ROS_INFO_STREAM(".....!!!.... ");
        moveToPose(T_B_Fee_, p);
        lastPose = p;
        r.sleep();
      }
      r.sleep();
    }


    r.sleep();

    return true;
  }

bool kill_this_process = false;

void SigIntHandler(int signal)
{
  kill_this_process = true;
  ROS_WARN_STREAM("SHUTDOWN SIGNAL RECEIVED");
}

int main(int argc, char **argv)
{
  // Ros related
  ros::init(argc, argv, "asar_grasper");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigIntHandler);

  AsarHybridTmp at(node_handle, &kill_this_process);

  ros::AsyncSpinner spinner(8);
  spinner.start();

  std::thread t(std::bind(&AsarHybridTmp::controlLoop, &at));

  t.join();
  spinner.stop();

  return 0;
}
