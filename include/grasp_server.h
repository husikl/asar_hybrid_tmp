//* C
#include <signal.h>

//* C++
#include <thread>

//* External
// Pinocchio
#include <pinocchio/algorithm/frames.hpp>

// Eigen
#include <Eigen/Dense>
// // Eigen conversions
// #include <eigen_conversions/eigen_msg.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

// Actions
#include <actionlib/client/simple_action_client.h>


#include "asar_control/FollowCartesianTargetAction.h"
#include "asar_control/FollowJointTargetAction.h"
#include <asar_control/SolveIKAction.h>

#include <geometry_msgs/PoseArray.h>

#include <asar_hybrid_tmp/isPoseReachable.h>
#include <asar_hybrid_tmp/getTrajAction.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <asar_hybrid_tmp/getPath.h>
#include <asar_hybrid_tmp/GraspAction.h>
#include <asar_hybrid_tmp/PathFollowerAction.h>
#include <actionlib/server/simple_action_server.h>

#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <asar_hybrid_tmp/isHolding.h>
#include <asar_hybrid_tmp/getGraspSamples.h>
#include <asar_hybrid_tmp/getObjectPose.h>
#include <asar_hybrid_tmp/PddlMotions.h>
#include <asar_hybrid_tmp/SuturePath.h>
#include <asar_hybrid_tmp/SuturePoints.h>

#include <iostream>
#include <cmath>
#include <tuple>
#include <bits/stdc++.h>
#include <iomanip>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ros/package.h>
#include <vector>
#include <random>
#include <algorithm>
#include <unordered_set>
#include <asar_hybrid_tmp/checkIfPartnerCanGrasp.h>
#include <asar_hybrid_tmp/collisionCheck.h>
#include <asar_hybrid_tmp/GraspTrajectory.h>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <unordered_map>
#include <vector>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <asar_hybrid_tmp/AddNoise.h>



std::vector<int> generateRandomUniqueIntegers(int numValues, int lowerRange, int upperRange) {
    std::mt19937_64 generator(std::random_device{}());
    std::uniform_int_distribution<int> distribution(lowerRange, upperRange);
    std::unordered_set<int> uniqueIntegers;
    std::vector<int> uniqueIntegersVec;

    while (uniqueIntegersVec.size() < numValues) {
        int randomInteger = distribution(generator);
        if (uniqueIntegers.find(randomInteger) == uniqueIntegers.end()) {
          uniqueIntegers.insert(randomInteger);
          uniqueIntegersVec.push_back(randomInteger);
        }
    }
    return uniqueIntegersVec;
}




bool operator==(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs)
{ return std::tie(lhs.x, lhs.y) == std::tie(rhs.x, rhs.y); }

enum result_category { NONE, ONE_COINCEDENT, ONE_DIAMETER, TWO, INFINITE };

using result_t = std::tuple<result_category, geometry_msgs::Point, geometry_msgs::Point>;

double distance(geometry_msgs::Point l, geometry_msgs::Point r)
{ return std::hypot(l.x - r.x, l.y - r.y); }

result_t find_circles(geometry_msgs::Point p1, geometry_msgs::Point p2, double r)
{
    geometry_msgs::Point ans1, ans2; 
    if (p1 == p2) {
        if(r == 0.) return std::make_tuple(ONE_COINCEDENT, p1,   p2  );
        else        return std::make_tuple(INFINITE,       ans1, ans2);
    }
    geometry_msgs::Point center;
    center.x = p1.x/2 + p2.x/2;
    center.y = p1.y/2 + p2.y/2;  

    double half_distance = distance(center, p1);
    if(half_distance > r)      return std::make_tuple(NONE,         ans1,   ans2);
    if(half_distance - r == 0) return std::make_tuple(ONE_DIAMETER, center, ans2);
    double root = std::hypot(r, half_distance) / distance(p1, p2);
    ans1.x = center.x + root * (p1.y - p2.y);
    ans1.y = center.y + root * (p2.x - p1.x);
    ans2.x = center.x - root * (p1.y - p2.y);
    ans2.y = center.y - root * (p2.x - p1.x);
    return std::make_tuple(TWO, ans1, ans2);
}



// using namespace asar_ns;
namespace pin = pinocchio;

using namespace Eigen;
// using namespace cv;
using namespace std;
typedef actionlib::SimpleActionClient<asar_control::FollowCartesianTargetAction>
    FollowCartesianTargetClient;
typedef actionlib::SimpleActionClient<asar_control::FollowJointTargetAction>
    FollowJointTargetClient;

typedef actionlib::SimpleActionClient<asar_control::SolveIKAction>
    SolveIKClient;

typedef actionlib::SimpleActionClient<
    asar_hybrid_tmp::getTrajAction>
    TrajClient;

// Function to calculate the offset between two poses A and B
pin::SE3 calculateOffset(const pin::SE3& A, const pin::SE3& B) {
  // Calculate the inverse of pose A
  pin::SE3 A_inv = A.inverse();

  // Calculate the offset between A and B
  pin::SE3 offset_AB = A_inv * B;
  // offset_AB.rotation().normalize();
  return offset_AB;
}

void poseSE3ToMsg(const pin::SE3 &e, geometry_msgs::Pose &m)
{
  m.position.x = e.translation()[0];
  m.position.y = e.translation()[1];
  m.position.z = e.translation()[2];
  Eigen::Quaterniond q = (Eigen::Quaterniond)e.rotation();
  q.normalize();
  m.orientation.x = q.x();
  m.orientation.y = q.y();
  m.orientation.z = q.z();
  m.orientation.w = q.w();
  if (m.orientation.w < 0)
  {
    m.orientation.x *= -1;
    m.orientation.y *= -1;
    m.orientation.z *= -1;
    m.orientation.w *= -1;
  }
}

void transformSE3ToMsg(const pin::SE3 &e, geometry_msgs::Transform &m)
{
  m.translation.x = e.translation()[0];
  m.translation.y = e.translation()[1];
  m.translation.z = e.translation()[2];
  Eigen::Quaterniond q = (Eigen::Quaterniond)e.rotation();
  m.rotation.x = q.x();
  m.rotation.y = q.y();
  m.rotation.z = q.z();
  m.rotation.w = q.w();
  if (m.rotation.w < 0)
  {
    m.rotation.x *= -1;
    m.rotation.y *= -1;
    m.rotation.z *= -1;
    m.rotation.w *= -1;
  }
}

void poseMsgToSE3(const geometry_msgs::Pose &m, pin::SE3 &e)
{
  e = pin::SE3(pin::SE3::Quaternion(m.orientation.w, m.orientation.x,
                                    m.orientation.y, m.orientation.z),
               pin::SE3::Vector3(m.position.x, m.position.y, m.position.z));
}

void transformMsgToSE3(const geometry_msgs::Transform &m, pin::SE3 &e)
{
  e = pin::SE3(
      pin::SE3::Quaternion(m.rotation.w, m.rotation.x, m.rotation.y,
                           m.rotation.z),
      pin::SE3::Vector3(m.translation.x, m.translation.y, m.translation.z));
}





static const std::string OPENCV_WINDOW = "Image window";

class AsarHybridTmp
{
  private:
  //  ROS
  ros::NodeHandle nh_;
  // ros::Subscriber sub_T_Biface_Fiface_;
  ros::Subscriber sub_asar_ee_pose_;
  // ros::Subscriber            sub_simul_pose_;
  ros::Subscriber sub_image_;
  ros::Subscriber sub_needle_pose_;
  ros::Subscriber sub_needle_offs_;
  ros::Subscriber sub_needle_tip_;
  ros::Publisher  pub_T_B_Fee_cmd_;
  ros::Publisher  pub_asar_grip_cmd_;
  ros::Publisher  pub_grip_trigger_;
  // ros::Publisher pub_needle_dynamics_;
  ros::Publisher pub_grasps_counter_;
  // for debug only 
  ros::Publisher pub_needle_pose_;
  ros::Subscriber sub_arm_tf_;
  ros::Subscriber sub_ring_pose_;

  std::shared_ptr<
      actionlib::SimpleActionServer<asar_hybrid_tmp::GraspAction>>
      graspActionServ_;
  
  std::shared_ptr<
      actionlib::SimpleActionServer<asar_hybrid_tmp::PathFollowerAction>>
      pathFollowerServ_;

  ros::Subscriber            sub_contact_info_;

  ros::Subscriber            sub_tissue_points;

  geometry_msgs::PoseStamped pub_T_B_Fee_cmd_msg_;
  ros::Publisher pub_expectedPose;
  ros::Publisher pub_actualPose;
  std_msgs::Float32          pub_asar_grip_cmd_msg_;

  FollowCartesianTargetClient ac_follow_cart;
  FollowJointTargetClient     ac_follow_joint;
  SolveIKClient               ac_solveIK;

  // TrajClient ac_traj;

  vector<geometry_msgs::Point> path;
  // for python node
  ros::ServiceServer service1;

  ros::ServiceServer service2;
  
  ros::ServiceServer suture_ik_service_;
  ros::ServiceServer sample_pose_service;
  ros::ServiceServer get_obj_pose_service, motion_handler_service;
  ros::ServiceClient omplClient, omplClientWayps;
  ros::ServiceClient suturePathClient, suturePointsClient, handoverReachabilityClient;
  ros::ServiceClient collisionCheckerClient;
  ros::ServiceClient addNoiseClient_;
  // Frames
  double   iface_grip_angle_;
  pin::SE3 T_B_Bcam_;
  // pin::SE3 T_Biface_Fiface_map;

  pin::SE3 T_B_Fee_, eePoseSaved, eePoseToMove, eePose2;
  pin::SE3 T_B_Fee_cmd_;

  pin::SE3 w1, w2, w3, mid;

  pin::SE3 offs0, offs1, offs5, offs4, tipOffset, tipReverseOffset;

  pin::SE3 ringPose, tipPose;

  pin::SE3            needle_wayp;
  pin::SE3            initialPose, initNeedlePose;
  geometry_msgs::Pose eefPose;
  Vector3d            Bcam_err_, err;
  Vector3d            Bcam_err_last;

  double delBcam;
  double delBcam_last;
  int    grasps_counter = 0;

  int counter = 0;

  // pin::SE3 T_B_Fee_map;

  Matrix3d R_B_Biface;

  // Matrix3d R_Fiface_Fee_;
  // pin::SE3 T_Fiface_Fee_;

  // ASAR Controller
  // std::unique_ptr<AsarControl> ac_;

  bool *kill_this_node_;
  bool  mapping_done_;

  bool   ikSolved;
  bool   execute;
  bool   contactMade;
  double last_time_;
  double cmd_time_;
  // int    iface_button_;

  // Workspace limit
  Vector3d ws_center_;
  double   lx_;
  double   ly_;
  double   lz_;

  std::vector<double> S_ul_;
  std::vector<double> S_ll_;

  int loop_rate_;

  double storeLambda;
  bool   open_gripper;
  int    unit_id;

  geometry_msgs::PoseArray        needleWaypoints, tissuePoints;

  VectorXd q_solved;

  // tf2::Transform t1, t2;
  double storeAngle;
  
  boost::mutex mtx_Act_track_;
  Vector3d circleCenter;
  Vector3d circleNorm;

  enum
  {
    ACT_RESET = -1,
    ACT_NONE = 0,
    ACT_EXECUTE,
  };
  int          m_curAct_track_;

  string holding;
  pinocchio::Model model;

  Vector3d arms_tf_offset;

  int iterServicing;
  double safetyDistance;
  int samples_num_;
  int waypoints_num_;
  double ompl_timeout_;
  
  struct PoseAndJointValues
  {
    pin::SE3 pose;
    VectorXd jointValues;
    pin::SE3 offset;
  };

  struct GraspWaypoints
  {
    pin::SE3 grasp;
    vector<pin::SE3> waypoints;
    bool canReachAll = false;
  };

  struct MultiGraspTraj
  {
    int id;
    pin::SE3 grasp;
    vector<pin::SE3> traj;
  };

  vector<PoseAndJointValues> samplesPoses;


  vector<pin::SE3> graspCandidates;
  vector<pin::SE3> expectedGraspCandidates;
  vector<pin::SE3> usedGrasps;
  vector<pin::SE3> tipPoseTraj;
  vector<pin::SE3> tipPoseWaypoints;
  bool storeTipPose = false;

  pin::SE3 lastPose;

  vector<PoseAndJointValues> last_samples, last_sortedSamples;
  public:
  AsarHybridTmp(ros::NodeHandle &_node_handle, bool *kill_this_node);
  ~AsarHybridTmp();
  void controlLoop();

  // void SubGetInterfacePoseCb(const geometry_msgs::PoseStamped::ConstPtr
  // &msg);
  void SubGetAsarEEPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg);

  // void SubGetSimulationPoseCb(const geometry_msgs::PoseStamped::ConstPtr
  // &msg);
  // void SubPhantomButtons(const asar_control::PhantomButtonEvent::ConstPtr
  // &msg);
  void publishAsarEEPoseTarget();
  // void sendAsarEEPoseTarget(pin::SE3 Xd);
  void sendAsarJointTarget(VectorXd q_target);
  void sendSolverTarget(pin::SE3 Xd);
  void sendSolverAndExecute(pin::SE3 Xd);

  void limitWokspace(pin::SE3 &Xd);

  // void actFollowCartesianTargetDoneCb(
  //     const actionlib::SimpleClientGoalState &                 state,
  //     const asar_control::FollowCartesianTargetResultConstPtr &result);

  void actFollowJointTargetDoneCb(
      const actionlib::SimpleClientGoalState &             state,
      const asar_control::FollowJointTargetResultConstPtr &result);

  void actSolveIKDoneCb(const actionlib::SimpleClientGoalState &   state,
                        const asar_control::SolveIKResultConstPtr &result);

  bool isPoseIKReachable(
      asar_hybrid_tmp::isPoseReachable::Request & req,
      asar_hybrid_tmp::isPoseReachable::Response &res);
  

  void moveToPose(pin::SE3 currentPose, pin::SE3 targetPose);

  void changeZ(float z);

  void     subSimContactInfo(const std_msgs::String::ConstPtr &msg);
  void     subNeedleCb(const geometry_msgs::PoseArray::ConstPtr &msg);
  void     subOffsetsCb(const geometry_msgs::PoseArray::ConstPtr &msg);
  void     subTissPointsCb(const geometry_msgs::PoseArray::ConstPtr &msg);
  pin::SE3 applyRotationToPose(const pin::SE3& needlePose, double xAngle, double yAngle,
                        double zAngle);

  // void subArmTransform(const geometry_msgs::TransformStamped::ConstPtr &msg);
  void executeCB(const asar_hybrid_tmp::GraspGoalConstPtr &goal);

  void pathFollowerCB(const asar_hybrid_tmp::PathFollowerGoalConstPtr &goal);

  void subTipPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg);

  vector<double> equation_plane(Vector3d p1, Vector3d p2, Vector3d p3) 
  {
    // Matrix3d rotation;
    double a, b, c , d;
    Vector3d v1, v2, n;

    v1 = p2-p1;
    v2 = p3-p1;

    n = v1.cross(v2);
    a = n(0);
    b = n(1);
    c = n(2);
    d = (- a * p1(0) - b * p1(1) - c * p1(2));

    vector<double> res;
    res.push_back(a);
    res.push_back(b);
    res.push_back(c);
    res.push_back(d);
    return res;
  }



  void samplePosesFromNeedle(string type, double initAngle, int samplesNum, vector<PoseAndJointValues> &reachablePoses)
{
    // ROS_INFO("Sampling poses...");
    // ROS_INFO_STREAM("samplesNum = " << samplesNum);
    // ROS_INFO_STREAM("sample type = " << type);

    pin::SE3 sample_pose, sample_offset;
    double sampleAngle = 0;

    if (type == "insert")
    {
        sample_offset = offs5;
        initAngle = -M_PI / 10;
    }
    else if (type == "extract")
    {
        sample_offset = offs4;
        initAngle = -M_PI / 6;
    }
    else if (type == "grasp")
    {
        sample_offset = offs1;
        initAngle = -M_PI / 10;
    }
    else
    {
        sample_offset = offs1;
        initAngle = -M_PI / 10;
    }

    std::vector<int> z_axis_angles, x_axis_angles;

    // z_axis_angles = generateRandomUniqueIntegers(samplesNum, 0, 45);
    z_axis_angles = generateRandomUniqueIntegers(samplesNum, 0, 30);
    x_axis_angles = generateRandomUniqueIntegers(samplesNum, -30, 30);

    reachablePoses.reserve(samplesNum * samplesNum * 2);

    PoseAndJointValues sample;

    #pragma omp parallel for collapse(2)
    for (int z_i_ : z_axis_angles)
    {
        for (int x_i_ : x_axis_angles)
        {
            double zangle = (M_PI / 180) * z_i_;
            double xangle = (M_PI / 180) * x_i_;

            sampleAngle = initAngle + (M_PI / 180) * z_i_;
            sample_pose = applyRotationToPose(mid, 0, 0, sampleAngle);
            sample_pose = sample_pose * sample_offset;
            sample_pose = applyRotationToPose(sample_pose, xangle, 0, 0);
            sample_pose = applyRotationToPose(sample_pose, 0, zangle, 0);
            

            if (isPoseKinematicallyValid(sample_pose))
            {
                sample.pose = sample_pose;
                sample.jointValues = q_solved;
                sample.offset = calculateOffset(sample_pose, tipPose);

                if (checkIfPoseCollisionFree(sample))
                {
                    #pragma omp critical
                    {
                        reachablePoses.push_back(sample);
                    }
                }
                // else
                // {
                //   ROS_WARN("not coll-n free ");
                // }
            }
          // else
          //   {
          //     ROS_INFO("not ik valid ...?");
          //   }
            // sample_pose = applyRotationToPose(sample_pose, 0, 0, M_PI);
            sample_pose = applyRotationToPose(sample_pose, 0, M_PI, 0);

            if (isPoseKinematicallyValid(sample_pose))
            {
                sample.pose = sample_pose;
                sample.jointValues = q_solved;
                sample.offset = calculateOffset(sample_pose, tipPose);

                if (checkIfPoseCollisionFree(sample))
                {
                    #pragma omp critical
                    {
                        reachablePoses.push_back(sample);
                    }
                }
            }
            // else
            // {
            //   ROS_INFO("2 not ik valid ...?");
            // }
        }
    }

    ROS_INFO_STREAM("unit_id "<< unit_id);
    ROS_INFO_STREAM("Poses size = " << reachablePoses.size());
}



void samplePosesFromExpectedNeedlePose(pin::SE3 expectedTipPose,string type, double initAngle, int samplesNum, vector<PoseAndJointValues> &reachablePoses);
  

void sortPosesByHighestManipulabilityScore(vector<PoseAndJointValues> &samples, vector<PoseAndJointValues> &sortedPoses )
{
    // Create a vector of indices into the samples vector
    vector<int> indices(samples.size());
    std::iota(indices.begin(), indices.end(), 0);

    // Sort the indices based on the manipulability scores of the corresponding elements
    sort(indices.begin(), indices.end(), [this, &samples](int i1, int i2) -> bool {
    return getManipulabilityScore(samples[i1].jointValues) > getManipulabilityScore(samples[i2].jointValues);
    });

    // Create the sortedPoses vector using the sorted indices
    sortedPoses.resize(samples.size());
    for (int i = 0; i < samples.size(); i++) {
        sortedPoses[i] = samples[indices[i]];
    }
}


// void getTopKSamplesByManipulabilityScore(std::vector<PoseAndJointValues>& samples, int k, string type) {
//     // Sort the samples based on the manipulability scores
//     std::nth_element(samples.begin(), samples.begin() + k, samples.end(), 
//                 [this](const PoseAndJointValues& a, const PoseAndJointValues& b) 
//               {
//                 return getManipulabilityScore(a.jointValues) > getManipulabilityScore(b.jointValues);
//               });
    
//     samples.resize(k);
//     if (type == "translation")
//     {
//       std::nth_element(samples.begin(), samples.begin() + k, samples.end(), 
//                 [this](const PoseAndJointValues& a, const PoseAndJointValues& b) 
//               {
//                 return getTranslationManipulability(a.jointValues) > getTranslationManipulability(b.jointValues);
//               });
//       return;
//     }

//     else if (type == "orientation")
//     {
//       std::nth_element(samples.begin(), samples.begin() + k, samples.end(), 
//                 [this](const PoseAndJointValues& a, const PoseAndJointValues& b) 
//               {
//                 return getOrientationManipulability(a.jointValues) > getOrientationManipulability(b.jointValues);
//               });
//       return;
//     }

//     else 
//       return;
// }



double getManipulabilityScore(Eigen::VectorXd jointValues) 
{
    Eigen::VectorXd q(model.nq);
    q << jointValues;  

    // Compute the Jacobian matrix for the end-effector
    pin::Data data(model);
    pin::computeJointJacobians(model, data, q);
    const Eigen::MatrixXd &J = data.J;

    // Compute the SVD of the Jacobian using JacobiSVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd singular_values = svd.singularValues();

    // Calculate the manipulability measure
    double manipulability = singular_values.minCoeff() / singular_values.maxCoeff();
    return manipulability;
}



  void graspControl(string command, pin::SE3 pose)
  {
    ros::Rate r(10.0);
    std_msgs::String s;
    T_B_Fee_cmd_ = T_B_Fee_;
    int angle = 1;
    if (command == "grasp")
    {
      ROS_WARN("grasp request ....");
      
      while (angle < 14)
      {
        iface_grip_angle_ = (20- angle)*M_PI/180.0;
        sendSolverAndExecute(pose);
        pub_asar_grip_cmd_msg_.data = iface_grip_angle_;
        pub_asar_grip_cmd_.publish(pub_asar_grip_cmd_msg_);
        pub_T_B_Fee_cmd_msg_.header.stamp = ros::Time::now();
        poseSE3ToMsg(T_B_Fee_cmd_, pub_T_B_Fee_cmd_msg_.pose);
        pub_T_B_Fee_cmd_.publish(pub_T_B_Fee_cmd_msg_);
        
        r.sleep();
        angle++;
        // ROS_INFO_STREAM("angle: " << angle);
        
      }
      // s.data = "dynamic";
      // pub_needle_dynamics_.publish(s);
      sleep(1.0);
      s.data = "grasp";
      for (int i = 0; i < 5; i++)
      {
        pub_grip_trigger_.publish(s);
        r.sleep();        
      }

      // turn off for pddlstream?
      grasps_counter++;
      ROS_INFO_STREAM("counter = " << grasps_counter);
      

      open_gripper = false;
      if (unit_id == 1)
        holding = "unit1";
      if (unit_id == 0)
        holding = "unit0";

      
    }

    else if (command == "open")
    {
        ROS_WARN("open request ....");
        s.data = "open";
        for (int i = 0; i < 5; i++)
        {
        pub_grip_trigger_.publish(s);
        sendSolverAndExecute(T_B_Fee_cmd_);
        // publishAsarEEPoseTarget();
        // r.sleep();        
        }

        sendSolverAndExecute(T_B_Fee_cmd_);
        // publishAsarEEPoseTarget();
        while (angle < 8)
        {
        iface_grip_angle_ += angle*M_PI/180.0;
        sendSolverAndExecute(T_B_Fee_cmd_);
        pub_asar_grip_cmd_msg_.data = iface_grip_angle_;
        pub_asar_grip_cmd_.publish(pub_asar_grip_cmd_msg_);
        pub_T_B_Fee_cmd_msg_.header.stamp = ros::Time::now();
        poseSE3ToMsg(T_B_Fee_cmd_, pub_T_B_Fee_cmd_msg_.pose);
        pub_T_B_Fee_cmd_.publish(pub_T_B_Fee_cmd_msg_);
        
        r.sleep();
        angle++;
        // ROS_INFO_STREAM("angle: " << angle);
        
        }
        open_gripper = true;
        // holding = "";
      
        holding = "";
    }
    else
    {
      open_gripper = false;
      // ROS_INFO_STREAM("command = " << command);
      while (angle < 14)
      {
        iface_grip_angle_ = (20- angle)*M_PI/180.0;
        if (isPoseKinematicallyValid(pose))
          sendSolverAndExecute(pose);
        pub_asar_grip_cmd_msg_.data = iface_grip_angle_;
        pub_asar_grip_cmd_.publish(pub_asar_grip_cmd_msg_);
        pub_T_B_Fee_cmd_msg_.header.stamp = ros::Time::now();
        poseSE3ToMsg(T_B_Fee_cmd_, pub_T_B_Fee_cmd_msg_.pose);
        pub_T_B_Fee_cmd_.publish(pub_T_B_Fee_cmd_msg_);
        
        r.sleep();
        angle++;
        // ROS_INFO_STREAM("angle: " << angle);
        
      }
      // s.data = "dynamic";
      // pub_needle_dynamics_.publish(s);
      sleep(1.0);
    }

  }

  vector<pin::SE3> getPathWithOmpl(pin::SE3 startPose, pin::SE3 targetPose, int unit_id, string type, float &path_cost)
  {
    pin::SE3 s = normalizeOrientation(startPose);
    pin::SE3 g = normalizeOrientation(targetPose);
    path_cost = 0;
    if (lastPose.translation().norm() == 0)
    {
      ROS_WARN("lastPose is zero");
      poseMsgToSE3(eefPose, lastPose);
    }
    geometry_msgs::Pose needleCenter;
    // needleCenter = needleWaypoints.poses[3];

    std::chrono::steady_clock::time_point begin =
        std::chrono::steady_clock::now();

    ros::Rate r(loop_rate_);
    asar_hybrid_tmp::getPath omplReq;
    
    omplReq.request.speed = 0.01;
    if (type == "grasp")
    {
        omplReq.request.speed = 0.005;
    }

    if (type == "move")
    {
        omplReq.request.speed = 0.01;
    }
    omplReq.request.collisionObjects.push_back(needleCenter);
    omplReq.request.arm_index = std::to_string(unit_id);
    geometry_msgs::Pose targetP;


    poseSE3ToMsg(s, omplReq.request.start);

    poseSE3ToMsg(g, targetP);
    omplReq.request.goal = targetP;
    omplReq.request.time_out = ompl_timeout_;
    omplClient.call(omplReq);
    // ROS_INFO_STREAM("ompl plan : " << omplReq.response);
    if (omplReq.response.path.size() > 0)
    {
      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      // std::cout << "Time difference (sec) = "
      //           << (std::chrono::duration_cast<std::chrono::microseconds>(
      //                   end - begin)
      //                   .count()) /
      //                   1000000.0    << std::endl;
      vector<pin::SE3> trajectory;
      for (auto p : omplReq.response.path)
      {
        pin::SE3 waypoint;
        poseMsgToSE3(p, waypoint);
        trajectory.push_back(waypoint);
      }
      path_cost = omplReq.response.pathLength;
      return trajectory;
    }
    else 
    {
        ROS_WARN(" get Path no trajectory from ompl ...");
        return {};
    }
  }

  bool goToPoseWithOmpl(pin::SE3 targetPose, int unit_id, string type)
  {
    ros::Rate r(loop_rate_);

    if (lastPose.translation().norm() == 0)
    {
      ROS_WARN("lastPose is zero");
      poseMsgToSE3(eefPose, lastPose);
    }
    
    float cost;
    auto traj = getPathWithOmpl(lastPose, targetPose, unit_id, type, cost);
    if (traj.size() > 0)
    {
      for (auto waypoint : traj)
      {
        moveToPose(T_B_Fee_, waypoint);
        lastPose = waypoint;
        r.sleep();
      }

      return true;
    }
    
    return false;
  }

  void applyOffset(pin::SE3 &pose, double dist)
  {
    pin::SE3 testP;
    Matrix3d m;
    m.setIdentity();
    testP.translation().x() = 0;
    testP.translation().y() = dist;
    testP.translation().z() = 0;
    testP.rotation() = m;
    
    pose = pose * testP;
  }

  bool IsHolding(asar_hybrid_tmp::isHolding::Request  &req,
         asar_hybrid_tmp::isHolding::Response &res)
  {
    
    res.holding = false;
    res.arm_index = holding;
    if (unit_id == 0 && holding == "unit0")
    {
      res.holding = true;
      res.arm_index = holding;
    }
    else if (unit_id == 1 && holding == "unit1")
    {
      res.holding = true;
      res.arm_index = holding;
    }

    // ROS_WARN_STREAM("holding = " << holding);
    // ROS_WARN_STREAM("res = " << res);
    // sleep(30.0);


    return true;
  }

  bool getSamples(asar_hybrid_tmp::getGraspSamples::Request  &req,
         asar_hybrid_tmp::getGraspSamples::Response &res)
  {
    
    // ROS_INFO("get samples is called ..");
    // ROS_INFO_STREAM(" req location ~ " << req);
    
    vector<PoseAndJointValues> sortedSamples;
    vector<PoseAndJointValues> samples;
    // std::chrono::steady_clock::time_point begin =
    //     std::chrono::steady_clock::now();
    // samplePosesFromNeedle(req.action_type,M_PI/8, samples_num_, samples);

    samplePosesFromNeedle("insert",M_PI/8, samples_num_, samples);
    sortPosesByHighestManipulabilityScore(samples, sortedSamples);
    

    return true;
  }
  
  bool getObjectPoseCb(asar_hybrid_tmp::getObjectPose::Request  &req,
         asar_hybrid_tmp::getObjectPose::Response &res)
  {
    
      if (req.targetName == "gripper")
        res.pose = eefPose;
      else if (req.targetName == "needle") 
        poseSE3ToMsg(mid, res.pose);
      else 
      {
        ROS_WARN("unknown target name !");
      }
        
    return true;
  }


  bool motions_service_cb(asar_hybrid_tmp::PddlMotions::Request  &req,
         asar_hybrid_tmp::PddlMotions::Response &res)
  {
    float cost;
    // check reachability 
    if (req.type == "reachability")
    {
      // ROS_INFO("check reachability");
      // grasp and targetPose
      pin::SE3 grasp;
      poseMsgToSE3(req.grasp, grasp);
      if (req.location_id == -1)
        {
          res.IKReachable = true;
          ROS_INFO("checked for init location!");
          return true;
        }
      // given location check if ik reachable to insertion point
      Vector3d p_in, p_out;
      float angle;

      getInsertionPointsAndAngleFromId("insert", req.location_id, p_in, p_out, angle);

      vector<pin::SE3> insert_waypoints;
      insert_waypoints = generate_waypoints("insert", p_in, p_out, angle, waypoints_num_);
      bool reachable1, reachable2, reachable3;
      reachable1 =  isGraspValid(grasp, insert_waypoints[0], "insert", p_in, p_out);
      reachable2 =  isGraspValid(grasp, insert_waypoints[1], "insert", p_in, p_out);
      reachable3 =  isGraspValid(grasp, insert_waypoints[2], "insert", p_in, p_out);
      if (reachable1 && reachable2 && reachable3)
        res.IKReachable =  true;
      return true;
    }

    if(req.type == "grasp_check") 
    {
      if (unit_id == 0 && holding != "unit0")
      {
        return true;
      }
      else if (unit_id == 1 && holding != "unit1")
      {
        return true;
      }
      Vector3d p_in, p_out;
      float angle;

      getInsertionPointsAndAngleFromId("insert", req.location_id, p_in, p_out, angle);
      
      vector<pin::SE3> insert_waypoints;
      insert_waypoints = generate_waypoints("insert", p_in, p_out, angle, waypoints_num_);
      
      bool reachable1, reachable2, reachable3;
      reachable1 =  isGraspValid(T_B_Fee_, insert_waypoints[0], "insert", p_in, p_out);
      reachable2 =  isGraspValid(T_B_Fee_, insert_waypoints[1], "insert", p_in, p_out);
      reachable3 =  isGraspValid(T_B_Fee_, insert_waypoints[2], "insert", p_in, p_out);
      if (reachable1 && reachable2 && reachable3)
        res.IKReachable =  true;
      return true;
    }
    
    if (req.type == "insert")
    {
      // ROS_INFO("check reachability");
      // grasp and targetPose
      pin::SE3 g;
      poseMsgToSE3(req.grasp, g);
      if (req.location_id == -1)
      {
        res.IKReachable = true;
        ROS_INFO("checked for init location!");
        return true;
      }
      // given location check if ik reachable to insertion point
      Vector3d p_in, p_out;
      float angle;

      getInsertionPointsAndAngleFromId("insert", req.location_id, p_in, p_out, angle);

      vector<pin::SE3> insert_waypoints;
      insert_waypoints = generate_waypoints("insert", p_in, p_out, angle, waypoints_num_);
      bool can_reach1, can_reach2;
      can_reach1 = isGraspValid(g,insert_waypoints[0],"insert", p_in, p_out);
      can_reach2 = isGraspValid(g,insert_waypoints[1],"insert", p_in, p_out);
      res.IKReachable =  false;
      if (can_reach1 && can_reach2)
        res.IKReachable =  true;
      return true;
    }
        
    if (req.type == "extract")
    {
      // ROS_INFO("check reachability");
      // grasp and targetPose
      pin::SE3 g;
      poseMsgToSE3(req.grasp, g);
      if (req.location_id == -1)
        {
          res.IKReachable = true;
          ROS_INFO("checked for init location!");
          return true;
        }
      // given location check if ik reachable to insertion point
      Vector3d p_in, p_out;
      float angle;

      getInsertionPointsAndAngleFromId("extract", req.location_id, p_in, p_out, angle);
      
      pin::SE3 cOfMotion;
      vector<pin::SE3> extract_waypoints;
      extract_waypoints = generate_waypoints("extract", p_in, p_out, angle, waypoints_num_);
      res.IKReachable =  isGraspValid(g,extract_waypoints[0],"extract", p_in, p_out);
      return true;
    }

    if (req.type == "extract_grasp")
    {
      if (req.location_id == -1)
        {
          res.IKReachable = true;
          ROS_INFO("extarct grasp ....checked for init location!");
          return true;
        }
      // ROS_INFO("check reachability");
      // grasp and targetPose
      Vector3d p_in, p_out;
      float angle;

      getInsertionPointsAndAngleFromId("extract", req.location_id, p_in, p_out, angle);
      pin::SE3 cOfMotion;
      vector<pin::SE3> extract_waypoints;
      extract_waypoints = generate_waypoints("extract", p_in, p_out, angle, 20);
      PoseAndJointValues newGrasp;

      vector<PoseAndJointValues> new_samples, new_sorterd_samples, reachable_samples;

      pin::SE3 expectedTip = extract_waypoints[0];
      samplePosesFromExpectedNeedlePose(expectedTip, "extract",M_PI/6, samples_num_, new_samples);
      sortPosesByHighestManipulabilityScore(new_samples, new_sorterd_samples);
      
      for (PoseAndJointValues p : new_sorterd_samples)
      {
          // T_B_Fee_cmd_ = p.pose;
          // publishAsarEEPoseTarget();
          if (isGraspValidForExpectedPose(p, extract_waypoints[1], "extract", p_in, p_out) )
          {
              // T_B_Fee_cmd_ = p.pose;
              // publishAsarEEPoseTarget();
              // ROS_INFO("found a valid update");
              geometry_msgs::Pose geomPose;
              // ROS_WARN_STREAM("geomPose = " << p.pose);
              poseSE3ToMsg(p.pose, geomPose);
              // ROS_WARN_STREAM("geomPose = " << geomPose);
              res.reachableGrasps.push_back(geomPose); 
          }
      }
      return true;
    }
    

    // return move-holding-trajectory - basically to bring needle to insertion point?
    if (req.type == "move_holding")
    { 
      // ROS_INFO("search move_holding");
      pin::SE3 g, t_pose;
      poseMsgToSE3(req.grasp, g);

      // if need to return needle to init location ...
      if (req.location_id == -1)
      {
        ROS_ERROR("searching move_holding for init ...");
        // auto traj = getPathWithOmpl(g, initNeedlePose, unit_id, "move", cost);
        // for (auto pt : traj)
        // {
        //   geometry_msgs::Pose p;
        //   poseSE3ToMsg(pt, p);
        //   res.path.push_back(p);
        // }
        // res.pathLength = cost;
        return true;

      }

      Vector3d p_in, p_out;
      float angle;

      getInsertionPointsAndAngleFromId("insert", req.location_id, p_in, p_out, angle);
      vector<pin::SE3> insert_waypoints;

      insert_waypoints = generate_waypoints("insert", p_in, p_out, angle, waypoints_num_);
      // need to obtain the end effector pose as input to getPathWithOmpl
      pin::SE3 ofst = calculateOffset(g, tipPose);
      pin::SE3 targetGrasp = insert_waypoints[0]*ofst.inverse();
      T_B_Fee_cmd_ = insert_waypoints[0];
      publishAsarEEPoseTarget();
      ros:: Rate r(1);
      r.sleep();
      // sleep(3.0);
      auto traj = getPathWithOmpl(g, targetGrasp, unit_id, "grasp", cost);
      
      for (auto pt : traj)
      {
        geometry_msgs::Pose p;
        poseSE3ToMsg(pt, p);
        res.path.push_back(p);
      }
      res.pathLength = cost;
      return true;
    }

    // return trajectory for eef to pose
    if (req.type == "move")
    {
      // ROS_INFO("search for path move ...");
      
      pin::SE3 init_pose, t_pose;
      poseMsgToSE3(req.target_pose, t_pose);
      poseMsgToSE3(eefPose, init_pose);
      bool checkS, checkG;
      checkS = isPoseKinematicallyValid(init_pose);
      checkG = isPoseKinematicallyValid(t_pose);
      if (!(checkS && checkG))
        return true;
      auto traj = getPathWithOmpl(init_pose, t_pose, unit_id, "move", cost);
      for (auto pt : traj)
      {
        geometry_msgs::Pose p;
        poseSE3ToMsg(pt, p);
        res.path.push_back(p);
      }
      res.pathLength = cost;
      return true;
    }

    // return trajectory for eef to grasp needle
    if (req.type == "grasp")
    { 
      // ROS_INFO("search grasp path");
      pin::SE3 g, t_pose;
      poseMsgToSE3(req.target_pose, t_pose);
      poseMsgToSE3(eefPose, g);
      auto traj = getPathWithOmpl(g, t_pose, unit_id, "grasp", cost);
      for (auto pt : traj)
      {
        geometry_msgs::Pose p;
        poseSE3ToMsg(pt, p);
        res.path.push_back(p);
      }
      res.pathLength = cost;
      return true;
    }
    
    if (req.type == "insertion")
    {
      // ROS_INFO("search insertion path");
      if (req.location_id == -1)
        {
          res.IKReachable = true;
          ROS_INFO("insertion checked for init location!");
          return true;
        } 

      Vector3d p_in, p_out;
      float angle;

      getInsertionPointsAndAngleFromId("insert", req.location_id, p_in, p_out, angle);
      
      pin::SE3 g, offset;
      poseMsgToSE3(req.grasp, g);
      vector<pin::SE3> insert_waypoints;
      
      insert_waypoints = generate_waypoints("insert", p_in, p_out, angle, waypoints_num_);
      int grasps = 0;
      vector<GraspWaypoints> combined_path;
      // for insert offset is not used!
      bool hasPath = getGraspTrajectoryList(g, offset, insert_waypoints[0],  insert_waypoints, "insert", p_in, p_out, grasps, combined_path);
      if (!hasPath)
      {
        res.IKReachable = false;
        ROS_WARN("no updated grasp cant insert ...");
        return true;  
      }
      

      for (auto gw : combined_path)
      {
        asar_hybrid_tmp::GraspTrajectory gt;
        poseSE3ToMsg(gw.grasp, gt.grasp);
        for (auto w: gw.waypoints)
        {
          geometry_msgs::Pose p;
          poseSE3ToMsg(w, p);
          gt.trajectory.push_back(p);
        }
        
        res.combined_path.push_back(gt);
        
      }
      res.regrasps_num = combined_path.size();
      res.IKReachable = true;
      res.pathLength = combined_path.size();
      return true;
    }

    if (req.type == "extraction")
    {
      // ROS_INFO("search extraction path");
      if (req.location_id == -1)
        {
          res.IKReachable = true;
          ROS_INFO("extraction checked for init location!");
          return true;
        }  

      Vector3d p_in, p_out;
      float angle;

      getInsertionPointsAndAngleFromId("insert", req.location_id, p_in, p_out, angle);

      vector<pin::SE3> extract_waypoints, in_wayps;
      extract_waypoints = generate_waypoints("extract", p_in, p_out, angle, waypoints_num_);
      in_wayps = generate_waypoints("insert", p_in, p_out, angle, waypoints_num_);
      
      // assume that we need to resample a grasp here anyway, any grasp that is valid ?
      PoseAndJointValues extractInitGrasp;
      // pin::SE3 g;
      // bool has_update =false;
      // extractInitGrasp = updateUsingExpectedGraspPose(extract_waypoints[0],in_wayps.back(), has_update, "extract", p_in, p_out);
      // if (!has_update)
      // {
      //   res.IKReachable = false;
      //   ROS_WARN("no grasp update: cant extract ...");
      //   return true;  
      // }
      vector<GraspWaypoints> combined_path;
      int grasps = 0;
      poseMsgToSE3(req.grasp, extractInitGrasp.pose);
      bool hasPath = getGraspTrajectoryList(extractInitGrasp.pose, extractInitGrasp.offset, in_wayps.back(),  extract_waypoints, "extract", p_in, p_out, grasps, combined_path);
      if (!hasPath)
      {
        ROS_WARN("no path: cant extract ...");
        res.IKReachable = false;
        return true;  
      }
      for (auto gw : combined_path)
      {
        asar_hybrid_tmp::GraspTrajectory gt;
        poseSE3ToMsg(gw.grasp, gt.grasp);
        for (auto w: gw.waypoints)
        {
          geometry_msgs::Pose p;
          poseSE3ToMsg(w, p);
          gt.trajectory.push_back(p);
        }
        
        res.combined_path.push_back(gt);
        
      }
      res.regrasps_num = grasps;
      res.IKReachable = true;
      res.pathLength = combined_path.size();
      return true;
    }

    return false;
  }

  bool isPoseKinematicallyValid(pin::SE3 pose)
  {
    limitWokspace(pose);
    sendSolverTarget(pose);      
    if (ikSolved)
    {
      ikSolved = false;
      return true;

    }
    return false;
  }

  bool isGraspValid(pin::SE3 currentGrasp, pin::SE3 goalPose, string type, Vector3d p_in, Vector3d p_out)
  {
    pin::SE3 ofst = calculateOffset(currentGrasp, tipPose);
    pin::SE3 targetGrasp = goalPose*ofst.inverse();
    Vector3d testingPoint, diff;
    ros::Rate rat(1.0);

    Vector3d forDist;
    forDist = targetGrasp.translation() - currentGrasp.translation();
    if (forDist.norm() > 0.3)
      {
        

        return false;
      }

    if (isPoseKinematicallyValid(targetGrasp))
    {
      // std::cout << "diff = " << diff.norm() << std::endl;
      if (type == "insert")
      {
        // ROS_INFO("insert check ...");
        testingPoint = targetGrasp.translation();
        diff = testingPoint - p_in;
        // return diff.norm() > safetyDistance;
        // ROS_INFO_STREAM("testingPoint = " << testingPoint);
        // return testingPoint(2) > p_in(2);
        return (testingPoint(2) - p_in(2) ) > 0.005;
        return true;
      }

      else if (type == "extract")
      {
        // ROS_INFO("exract check ...");
        testingPoint = targetGrasp.translation();
        diff = testingPoint - p_out;
        // return diff.norm() > safetyDistance;
        return (testingPoint(2) - p_out(2) ) > 0.005;
        // return testingPoint(2) > p_out(2);
        return true;
      }

      return true;
    }
    // ROS_WARN("grasp not valid ...");
    return false;
  }



  // make it return false when it fails!!!
  bool sendGraspNeedle(string type, int unit_id)
  {
    ros::Rate pr(loop_rate_);
    ROS_WARN_STREAM("arm grasping needle " << unit_id); 
    std::vector<PoseAndJointValues> samples, sortedSamples;
    samplePosesFromNeedle(type,M_PI/8, samples_num_, samples);
    // sleep(5.0);
    bool ompl_success = false;
    sortPosesByHighestManipulabilityScore(samples, sortedSamples);
    ros::Rate lr(loop_rate_);
    if (sortedSamples.size() > 0)
    {
      for (auto candidatePose : sortedSamples)
      {
          pin::SE3 reachPose = candidatePose.pose;
          applyOffset(reachPose, 0.01);
          float cost;
          auto traj = getPathWithOmpl(lastPose, reachPose, unit_id, "move", cost);
          pr.sleep();

          if (traj.size() > 0)
          {         
            ompl_success = goToPoseWithOmpl(reachPose, unit_id, "move");
          }
          else
            continue;

          applyOffset(reachPose, -0.01);
          
          pr.sleep();
          if (ompl_success)
            ompl_success = goToPoseWithOmpl(reachPose, unit_id, "grasp");
          
          pr.sleep();
          if (ompl_success)
          {
            graspControl("grasp", reachPose);   
            // ROS_INFO("should be grasping ...");
              if (unit_id == 1)
                holding = "unit1";
              else
                holding = "unit0";
            return true;
          }

      }

    }

    holding = "";
    // ROS_WARN("sendGraspNeedle failed !");
    return false;
  }

  bool moveHolding(pin::SE3 goalPose, int unit_id, string type, Vector3d p_in, Vector3d p_out, bool &regrasped)
  {
    regrasped = false;
    float cost;
    // ROS_INFO("move holding the needle");
    
    pin::SE3 grasp = T_B_Fee_;
    // pin::SE3 needle_tip = tipPose;
    ros::Rate r(loop_rate_);
    ros::Rate pr(loop_rate_);

    
    bool foundUpdate = false;
  
    if (!isGraspValid(grasp, goalPose, type, p_in, p_out))
    {
      // ROS_WARN("goal pose is NOT kinematically valid !");
      // ROS_INFO_STREAM(" grasp " << grasp.translation());
      // ROS_INFO_STREAM(" goalPose " << goalPose.translation());
      grasp = updateGraspPose(goalPose, foundUpdate, type, p_in, p_out);
      
      if (!foundUpdate)
      {
        ROS_WARN("no updated pose found !");
        return false;
      }  
      graspControl("open", T_B_Fee_cmd_);
      r.sleep();
      bool pose_reached = false;

      for (auto g : graspCandidates)
      {
        // get trajectory first here and execute next ...
        grasp = g;
        auto traj = getPathWithOmpl(lastPose, g, unit_id, "move", cost);
        if (traj.size() > 0)
        {
          // pr.sleep();
          for (auto tp : traj)
          {
            moveToPose(T_B_Fee_, tp);
            r.sleep();
          }
          graspControl("grasp", lastPose);
          r.sleep();
          pose_reached = true;
          regrasped = true;
          break;

        }       

      }
      if (!pose_reached)
      {
        r.sleep();
        // ROS_ERROR("move holding ... failed to regrasp !!!!");
        return false;    
      }
    }

    // here targetToGo should be in arm_1 frame
    pin::SE3 ofst = calculateOffset(lastPose, tipPose);
    pin::SE3 targetToGo = goalPose*ofst.inverse();

    // here also get the trajectory first and execute after that,
    // that way i will know if ompl is failing or is it a smth else ...
    auto traj = getPathWithOmpl(lastPose, targetToGo, unit_id, "grasp", cost);
  
    if (traj.size() > 0)
    {
      for (auto tp : traj)
      {
        moveToPose(T_B_Fee_, tp);
        r.sleep();
      }
      return true;

    }        
    // ROS_ERROR("move holding : ompl failed !!!!");
    return false;

  }


  // function to sample a new grasp pose
  pin::SE3 updateGraspPose(pin::SE3 &goalPose, bool &updated, string type, Vector3d p_in, Vector3d p_out)
  {
    ros::Rate r(loop_rate_);
    pin::SE3 newGrasp;

    vector<PoseAndJointValues> samples, sortedSamples;
    sortedSamples.clear();
    samples.clear();
    samplePosesFromNeedle(type,M_PI/8, samples_num_, samples);
    sortPosesByHighestManipulabilityScore(samples, sortedSamples);
    // getTopKSamplesByManipulabilityScore(sortedSamples, 10, "orientation");
    graspCandidates.clear();
    for (PoseAndJointValues p : sortedSamples)
    {
      // T_B_Fee_cmd_ = p.pose;
      // publishAsarEEPoseTarget();
      if (isGraspValid(p.pose, goalPose, type, p_in, p_out) && graspCandidates.size() < 10)
      {
        // T_B_Fee_cmd_ = p.pose;
        // publishAsarEEPoseTarget();
        // ROS_INFO("found a valid update");
        newGrasp = p.pose;
        graspCandidates.push_back(p.pose);
        updated = true;
        // return newGrasp;
        
      }
      if (graspCandidates.size() >= 10)
        break;
      r.sleep();
    }
    if (graspCandidates.size() > 0)
    {
      updated = true;
      return graspCandidates[0];
    }
      
    // ROS_ERROR("no valid grasp updates !!!");
    updated=false;
  }

  // function to sample a new grasp pose using already available samples...
  pin::SE3 updateUsingSavedSamplesGraspPose(pin::SE3 &goalPose, bool &updated, string type, Vector3d p_in, Vector3d p_out)
  {
    ros::Rate r(loop_rate_);
    pin::SE3 newGrasp;

    if (last_samples.size() == 0)
    {
      samplePosesFromNeedle(type,M_PI/8, samples_num_, last_samples);
      sortPosesByHighestManipulabilityScore(last_samples, last_sortedSamples);
    }
    // getTopKSamplesByManipulabilityScore(sortedSamples, 10, "orientation");
    graspCandidates.clear();
    for (PoseAndJointValues p : last_sortedSamples)
    {
      // T_B_Fee_cmd_ = p.pose;
      // publishAsarEEPoseTarget();
      if (isGraspValid(p.pose, goalPose, type, p_in, p_out) && graspCandidates.size() < 10)
      {
        // T_B_Fee_cmd_ = p.pose;
        // publishAsarEEPoseTarget();
        // ROS_INFO("found a valid update");
        newGrasp = p.pose;
        graspCandidates.push_back(p.pose);
        updated = true;
        // return newGrasp;
        
      }
      if (graspCandidates.size() >= 15)
        break;
      r.sleep();
    }
    if (graspCandidates.size() > 0)
    {
      updated = true;
      return graspCandidates[0];
    }
      
    
    updated=false;
  }


  void getNeedleCenterGivenPoints(pin::SE3 &centerPoseForInsertion, Vector3d pIn, Vector3d pOut)
  {
    geometry_msgs::Point r1, r2, in1, ex1;
    result_t r;
    geometry_msgs::Pose needleCenter;

    Matrix3d rot;

    // ROS_INFO_STREAM("pIn: " << pIn.transpose());
    // ROS_INFO_STREAM("pOut: " << pOut.transpose());
    // ex2 = ex1;
    // ex2 = ex1.x + 0.05;
    Vector3d ex2;
    ex2 = pOut;
    ex2(0) = ex2(0) + 0.01;
    ex2(1) = ex2(1) + 0.01;
    vector<double > plane = equation_plane(pIn, pOut, ex2);
    

    // get first unit vector u1;  connecting two points on a circle
    Vector3d i1, e1, u1, t1, n_z, u2, normal;

    u1 = pOut - pIn;
    // u2 is made of a, b and c from plane constructed with 3 points;
    u2 << plane[0], plane[1], plane[2];

    normal = u1.cross(u2);

    Matrix3d rotation;
    normal.normalize();
    Vector2d nxy(normal(0), normal(1));
  
    rotation.setIdentity();
    rotation << normal(1)/nxy.norm(), -normal(0)/nxy.norm(),  0,
          normal(0)*normal(2)/nxy.norm(), normal(1)*normal(2)/nxy.norm(), -nxy.norm(),
          normal(0), normal(1), normal(2);
    
    // rotation.normalize();
    Eigen::AngleAxisd yawAngle(M_PI_2, Eigen::Vector3d::UnitZ());

    // why do we do this?
    // Eigen::AngleAxisd rollAngle(-M_PI_2, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());

    Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());

    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
    q.normalize();
    Eigen::Matrix3d rotationMatrix = q.matrix();
    // rotationMatrix.normalize();
    
    // Find coordinates of point in the plane
    Vector2d pointInPlane1_2d;
    Vector2d pointInPlane2_2d;
    u1.normalize();

    u2 = u1.cross(normal);


    pointInPlane1_2d(0) = pIn.dot(u1);
    // gives negative value ...
    pointInPlane1_2d(1) = pIn.dot(u2);
    pointInPlane2_2d(0) = pOut.dot(u1);
    // gives negative value ...
    pointInPlane2_2d(1) = pOut.dot(u2);
    // ROS_INFO("pointInPlane1 = %f,%f", pointInPlane1_2d(0), pointInPlane1_2d(1));
    // ROS_INFO("pointInPlane2 = %f,%f", pointInPlane2_2d(0), pointInPlane2_2d(1));



    // assumes that needle is being inserted w.r.t to y-z plane
    // probably need to change later ...
    in1.x = pointInPlane1_2d(0);
    in1.y = pointInPlane1_2d(1);

    ex1.x = pointInPlane2_2d(0);
    ex1.y = pointInPlane2_2d(1);

    r = find_circles(in1, ex1, 0.02);
    

    

    Eigen::Matrix3d R;
    Vector3d tempR;

    r2.x = (pIn(0) + pOut(0)) / 2;
    r2.y = (pIn(1) + pOut(1)) / 2;
    r2.z = std::get<1>(r).y;
    // why is there an offset?
    r2.z = r2.z - 0.008;
    if (r2.z < 0.0)
    {
      r2.z *= -1;
    }
    // tempR << r2.x,  r2.y, r2.z;
    // std::cout << "3D center for suturing: " << r2 << std::endl;
    // std::cout << "2nd r center: " << R * tempR << std::endl;
    
    // sleep(30.0);
    needleCenter.position = r2;
    needleCenter.orientation.w = 1.0;

    poseMsgToSE3(needleCenter, centerPoseForInsertion);

    // ROS_INFO_STREAM(" r2 ~ " << r2);
    centerPoseForInsertion.rotation() = rotation*rotationMatrix;
    // centerPoseForInsertion.rotation() = rotation;
  }

  bool checkIfPartnerCanGraspGivenExpectedNeedlePose(asar_hybrid_tmp::checkIfPartnerCanGrasp::Request  &req,
         asar_hybrid_tmp::checkIfPartnerCanGrasp::Response &res)
  {
  
    pin::SE3 needleTipPose;
    poseMsgToSE3(req.needlePose, needleTipPose);
    res.can_grasp = checkCanGraspGivenExpectedNeedlePoses(needleTipPose);
    return true;
  }

  void writePointsToFile(const std::string& sampling_type,const std::vector<pin::SE3>& waypoints,const std::vector<pin::SE3>& tipPoses, const std::string& type)
  {
      // Get the current time and format it as a string
      auto now = std::chrono::system_clock::now();
      std::time_t now_c = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");

      string filename_prefix;
      string wayp_prefix; 
      if (type == "insert")
      {
        filename_prefix = "insert_traj";
        wayp_prefix = "in";
      }

      if (type == "extract")
      {
        filename_prefix = "extract_traj";
        wayp_prefix = "out";
      }
      // Append the current date and time to the filename prefix
      std::string filename = filename_prefix + "_" + ss.str() + ".txt";

      // Prepend the directory path to the filename
      filename = "~/benchmark_suturing/ait_evaluations/" + filename;
      // Open the file for writing, creating it if it doesn't exist
      std::ofstream outfile;
      outfile.open(filename, std::ios::out | std::ios::trunc);

      // Set the output precision to 3 decimal places
      outfile << std::fixed << std::setprecision(6);
      
      // Write the unit ID as a comment at the beginning of the file
      outfile << "# Unit ID: " << unit_id << std::endl;
      outfile <<  "# number of samples: " << samples_num_ << std::endl;
      outfile << "# waypoints num : " << waypoints_num_ << std::endl;
      outfile << "# ompl timeout : " << ompl_timeout_ << std::endl;
      outfile << "# sampling_type : " << sampling_type << std::endl;
      
      outfile << " " << std::endl;
      outfile << " " << wayp_prefix << " waypoints trajectory" << std::endl;
      outfile << "x      y      z" << std::endl;

      // Iterate over the points and write their x, y, and z positions to the file in columns
      for (const auto& w : waypoints)
      {
        Vector3d wp;
        wp = w.translation();
        // if (unit_id == 1)
        //   wp +=  arms_tf_offset;
        outfile << std::setw(7) << wp(0) << " " << std::setw(7) << wp(1) << " " 
          << std::setw(7) << wp(2) << std::endl;
      }

      outfile << "                " << std::endl;
      outfile << " " << wayp_prefix  <<" Tip Pose trajectory" << std::endl;
      outfile << "x      y      z" << std::endl;

      // Iterate over the points and write their x, y, and z positions to the file in columns
      for (const auto& p : tipPoses)
      {
          outfile << std::setw(7) << p.translation().x() << " " << std::setw(7) << p.translation().y() << " " 
          << std::setw(7) << p.translation().z() << std::endl;
      }

      // Close the file
      outfile.close();
  }

  vector<pin::SE3> generate_waypoints(string type, Vector3d p_in, Vector3d p_out, const double angle, int w_num_)
  {
    pin::SE3 needlePose, targetCenterPose;
    getNeedleCenterGivenPoints(needlePose, p_in, p_out);
    pin::SE3 useOffset = tipOffset;
    vector<pin::SE3> zero_wayps;
    // vector<pin::SE3> wayps_offsets;
    waypoints_num_  = w_num_;
    int insertDegrees = 125; //135

    int j = round<int>(insertDegrees/w_num_);
    

    double startAngle = 0;
    if (type == "insert")
      startAngle = 55.0*(M_PI/180.0);
    else if (type == "extract")
      startAngle = 200.0*(M_PI/180.0);
    else
    {
      ROS_ERROR("Wrong type of trajectory");
      return {};
    }

    vector<pin::SE3> waypoints;
    ros::Rate r(1);
    // T_B_Fee_cmd_ = needlePose;
    // publishAsarEEPoseTarget();
    // r.sleep();
    // sleep(3);
    if (angle == 0)
    {
      for (double i = -j; i <= insertDegrees; i = i + j )
      {
        
        pin::SE3 goToPose;
        pin::SE3 centerPoseUpdated;
        double sampleAngle = (M_PI/ 180) * i + startAngle;
        centerPoseUpdated = applyRotationToPose(needlePose, 0, 0, sampleAngle);
        goToPose = centerPoseUpdated*useOffset;
        zero_wayps.push_back(normalizeOrientation(goToPose));
        // wayps_offsets.push_back(calculateOffset(goToPose, needlePose));
        // T_B_Fee_cmd_ = goToPose;
        // publishAsarEEPoseTarget();
        // r.sleep();
      }
      return zero_wayps;
    }

    else 
    {
      
      targetCenterPose = applyRotationToPose(needlePose, M_PI - angle, 0, 0);
      // targetCenterPose.translation() = needlePose.translation();
      // T_B_Fee_cmd_ = targetCenterPose;
      // publishAsarEEPoseTarget();
      r.sleep();
      // ROS_INFO("check center ... Waiting for 5 seconds");
      // sleep(3.0);
      for (double i = -j; i <= insertDegrees; i = i + j )
      {
        
        pin::SE3 goToPose;
        pin::SE3 centerPoseUpdated;
        double sampleAngle = (M_PI/ 180) * i + startAngle;
        centerPoseUpdated = applyRotationToPose(targetCenterPose, 0, 0, sampleAngle);
        goToPose = centerPoseUpdated*useOffset;
        // zero_wayps.push_back(goToPose);
        // wayps_offsets.push_back(calculateOffset(goToPose, needlePose));
        waypoints.push_back(goToPose);
        // T_B_Fee_cmd_ = centerPoseUpdated;
        // publishAsarEEPoseTarget();
        // r.sleep();
        // Vector3d dist1 = needlePose.translation() - goToPose.translation();
        // Vector3d dist2 = targetCenterPose.translation() - goToPose.translation();
        // ROS_INFO_STREAM("dist 1 " << i << " : " << dist1.norm());
        // ROS_INFO_STREAM("dist 2 " << i << " : " << dist2.norm());

      }

      return waypoints;
    }
  }

  bool isGraspValidForExpectedPose(PoseAndJointValues& currentGrasp, pin::SE3& goalPose, string type, Vector3d p_in, Vector3d p_out);
  bool canReachToTipPose(pin::SE3 startNeedleP, pin::SE3 goalNeedleP, pin::SE3 offset, vector<pin::SE3>& traj);
  PoseAndJointValues updateUsingExpectedGraspPose(pin::SE3 &goalPose, pin::SE3 &expectedTip, bool &updated, string type, Vector3d p_in, Vector3d p_out);
  bool getGraspTrajectoryList(pin::SE3 grasp, pin::SE3 startOffset, pin::SE3 expectedTip, vector<pin::SE3> waypoints,
         string type, Vector3d p_in, Vector3d p_out, int &regrasp_num, vector<GraspWaypoints> &combined_path);
  bool executeGraspTrajectoryList(vector<GraspWaypoints> grasps);
  
  pin::SE3 normalizeOrientation(const pin::SE3& se3)
  {
      Eigen::Matrix3d R = se3.rotation();
      Eigen::Quaterniond q(R);
      q.normalize();
      return pin::SE3(q, se3.translation());
  }


  bool followWaypoints(vector<pinocchio::SE3> waypoints, string type, Vector3d p_in, Vector3d p_out, int& regrasps)
{
    regrasps = 0;

    // Calculate the distance from the tip to the first waypoint
    double firstWaypointDistance = (waypoints[0].translation() - tipPose.translation()).norm();

    // If the distance to the first waypoint is greater than 0.02 m, set the closest waypoint to the first waypoint
    auto closestWaypoint = waypoints.begin();
    if (firstWaypointDistance <= 0.02) {
        // Find the closest waypoint to the current tip pose
        closestWaypoint = std::min_element(waypoints.begin(), waypoints.end(), [&](const pinocchio::SE3& wp1, const pinocchio::SE3& wp2) {
            double distance1 = (wp1.translation() - tipPose.translation()).norm();
            double distance2 = (wp2.translation() - tipPose.translation()).norm();
            return distance1 < distance2;
        });
    }

    // Start execution from the closest waypoint onwards
    for (auto it = closestWaypoint; it != waypoints.end(); ++it)
    {
        ros::Rate r(loop_rate_);
        bool regrasped = false;
        pinocchio::SE3& waypoint = *it; // Dereference the iterator

        bool reached = moveHolding(waypoint, unit_id, type, p_in, p_out, regrasped);

        if (!reached)
            return false;

        r.sleep();

        if (regrasped)
            regrasps++;
    }

    return true;
}

  std::vector<MultiGraspTraj> combineTrajectories(const std::vector<MultiGraspTraj>& input)
  {
    std::vector<MultiGraspTraj> result;
    
    for (const MultiGraspTraj& element : input)
    {
      bool found = false;
      
      for (MultiGraspTraj& combined : result)
      {
        if (element.id == combined.id)
        {
          // Combine trajectories
          combined.traj.insert(combined.traj.end(), element.traj.begin(), element.traj.end());
          found = true;
          break;
        }
      }
      
      if (!found)
      {
        // Add new element to result vector
        result.push_back(element);
      }
    }
    
    return result;
  }

  bool handoverRegionSampler()
  {

    pin::SE3 currentGrasp = T_B_Fee_;
    pin::SE3 sample_pose;
    ros::Rate r(100);

    // currentGrasp.translation().z() += 0.01;
    pin::SE3 graspToNeedleOffset = calculateOffset(currentGrasp, tipPose);
    for (int i = -360; i < 360; i += 15)
    {
      for (int j = -360; j< 360; j += 15)
      {
        //rotation around y-axis
        int yaw = (M_PI/ 180) * i;
        // rotation aroudn x-axis
        int pitch = (M_PI/ 180) * j;
        pin::SE3 offset;
        // could potentially add sampling around x as well..., for now only around z angle.
        sample_pose = applyRotationToPose(currentGrasp, 0, 0, yaw);
        sample_pose = applyRotationToPose(sample_pose, pitch, 0, 0);

        // double dz = ((double)rand() / RAND_MAX) * 0.04-0.02;           // Random value between 0 and 0.02
        // double dx = ((double)rand() / RAND_MAX) * 0.04 - 0.02;    // Random value between -0.02 and 0.02
        // double dy = ((double)rand() / RAND_MAX) * 0.02 - 0.02; 

        // sample_pose.translation().x() += dx;
        // sample_pose.translation().y() += dy;
        // sample_pose.translation().z() += dz;
        Vector3d p_in, p_out;
        // ROS_INFO_STREAM(" i = " << i);
        // T_B_Fee_cmd_ = sample_pose;
        // publishAsarEEPoseTarget();
        // r.sleep();
        if (isPoseKinematicallyValid(sample_pose))
        {         
          // next check if partner arm can reach it ...? here it should be expected needlePose shared or not ...?

          asar_hybrid_tmp::checkIfPartnerCanGrasp srv;
          pin::SE3 needleP = sample_pose*graspToNeedleOffset;
          T_B_Fee_cmd_ = needleP;
          publishAsarEEPoseTarget();
          r.sleep();
          poseSE3ToMsg(needleP, srv.request.needlePose);
          handoverReachabilityClient.call(srv);
          if (!srv.response.can_grasp)
            continue;
          
          // here also get the trajectory first and execute after that,
          // that way i will know if ompl is failing or is it a smth else ...
          float cost;
          auto traj = getPathWithOmpl(T_B_Fee_, sample_pose, unit_id, "move", cost);
        
          if (traj.size() > 0)
          {
              

            for (auto tp : traj)
            {
              moveToPose(T_B_Fee_, tp);
              r.sleep();
            }
          
            ROS_INFO("handover region reached ..?");
            return true;

          }
        }
      }
     

    }
    ROS_WARN("cant reach handover region..?");
    return false;
  }

  bool randomizedHandoverRegionSampler(int maxAttempts)
  {
      pin::SE3 currentGrasp = T_B_Fee_;
      pin::SE3 sample_pose;
      ros::Rate r(100);

      pin::SE3 graspToNeedleOffset = calculateOffset(currentGrasp, tipPose);

      // Generate random indices for sampling order
      std::vector<int> indices;
      for (int k = 0; k < 576; k++)
          indices.push_back(k);
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(indices.begin(), indices.end(), g);

      std::unordered_set<int> sampledIndices;  // Track sampled indices

      int numAttempts = 0; // Current number of attempts

      #pragma omp parallel
      {
          std::vector<std::pair<pin::SE3, bool>> validSamples;
          validSamples.reserve(maxAttempts);

          while (numAttempts < maxAttempts)
          {
              int k = indices[numAttempts % indices.size()];

              // Skip if the index has already been sampled
              if (sampledIndices.count(k) > 0)
              {
                  numAttempts++;
                  continue;
              }

              // Mark the index as sampled
              sampledIndices.insert(k);

              double yaw = (M_PI / 180.0) * (k % 24 * 15 - 180);
              double pitch = (M_PI / 180.0) * (k / 24 * 15 - 180);
              double roll = (M_PI / 180.0) * ((k / 24) % 2 == 0 ? 0 : 180);

              double dz = ((double)rand() / RAND_MAX) * 0.04-0.02;           // Random value between 0 and 0.02
              double dx = ((double)rand() / RAND_MAX) * 0.04 - 0.02;    // Random value between -0.02 and 0.02
              double dy = ((double)rand() / RAND_MAX) * 0.04;    // Random value between -0.02 and 0.02

              sample_pose = currentGrasp;
              applyRotationToPose(sample_pose, pitch, roll, yaw);
              sample_pose.translation().x() += dx;
              sample_pose.translation().y() += dy;
              sample_pose.translation().z() = currentGrasp.translation().z() + dz;

              // T_B_Fee_cmd_ = sample_pose;
              // publishAsarEEPoseTarget();
              // r.sleep();

              bool isPoseValid = isPoseKinematicallyValid(sample_pose);

              #pragma omp critical
              {
                  validSamples.emplace_back(sample_pose, isPoseValid);
              }

              #pragma omp barrier

              if (isPoseValid)
              {
                  pin::SE3 needleP = sample_pose * graspToNeedleOffset;
                  T_B_Fee_cmd_ = needleP;
                  publishAsarEEPoseTarget();
                  r.sleep();

                  asar_hybrid_tmp::checkIfPartnerCanGrasp srv;
                  poseSE3ToMsg(needleP, srv.request.needlePose);
                  handoverReachabilityClient.call(srv);

                  if (srv.response.can_grasp)
                  {
                      float cost;
                      auto traj = getPathWithOmpl(T_B_Fee_, sample_pose, unit_id, "move", cost);

                      if (!traj.empty())
                      {
                          #pragma omp critical
                          {
                              for (auto tp : traj)
                              {
                                  moveToPose(T_B_Fee_, tp);
                                  r.sleep();
                              }
                              ROS_INFO("Handover region reached.");
                              return true;
                          }
                      }
                  }
              }

              #pragma omp barrier
              #pragma omp single
              {
                  numAttempts++;
              }
          }
      }

      ROS_WARN("Unable to reach handover region within the maximum number of attempts.");
      return false;
  }



  //need to get the id to call the suturePOints dictionary and obtain the position of suturePoints at angle= 0
  //should just return the 
  void extractDigits(int n, int& referenceInteger) 
  {
    // Check if the input is a two-digit number
    if (!(n >= 10 && n <= 99)) {
        throw std::invalid_argument("Input is not a two-digit number.");
    }

    // Extract the first and second digits
    referenceInteger = n / 10;
    referenceInteger = referenceInteger*10;
}

  void getInsertionPointsAndAngleFromId(string type, int idNumber, Vector3d& in, Vector3d& out, float& angle)
  {
    asar_hybrid_tmp::SuturePath srv;
    srv.request.type = type;
    int referenceInteger;
    extractDigits(idNumber, referenceInteger);
    srv.request.id = referenceInteger;
    suturePathClient.call(srv);
    in << srv.response.p_in.x, srv.response.p_in.y, srv.response.p_in.z;
    out << srv.response.p_out.x, srv.response.p_out.y, srv.response.p_out.z;

    if (unit_id == 1)
    {
      in = in + arms_tf_offset;
      out = out + arms_tf_offset;
    }
    srv.request.id = idNumber;
    suturePathClient.call(srv);
    angle = srv.response.angle;
  }

  
  void goToPosition(pin::SE3 &targetPosition)
  {
    // ROS_WARN("sample orientations to reach the position ");
    if (isPoseKinematicallyValid(targetPosition) )
    {
      moveToPose(T_B_Fee_, targetPosition);
      return;
    }
    #pragma omp parallel for collapse(2)
    for (int y_i_ = 0; y_i_ < 360; ++y_i_) 
    {
        for (int x_i_ = 0; x_i_ < 360; ++x_i_) 
        {
            double yangle = (M_PI / 180) * y_i_;
            double xangle = (M_PI / 180) * x_i_;

            pin::SE3 sample_pose = targetPosition;
            sample_pose = applyRotationToPose(sample_pose, xangle, 0, 0);
            sample_pose = applyRotationToPose(sample_pose, 0, 0, yangle);
            

            if (isPoseKinematicallyValid(sample_pose) )
            {
  
              #pragma omp critical
              {
                  targetPosition = sample_pose;
                  moveToPose(T_B_Fee_, sample_pose);
                  return;
              }
            }


        }
    }
    
  }

  bool checkCanGraspGivenExpectedNeedlePoses(pin::SE3 expectedTipPose)
  {
      // ROS_WARN("Sample pose for handover called...");
      vector<PoseAndJointValues> new_samples;
      if (unit_id == 1)
        expectedTipPose.translation() += arms_tf_offset;
      // T_B_Fee_cmd_ = expectedTipPose;
      // publishAsarEEPoseTarget();
      // ros::Rate r(10);
      // r.sleep();      
      samplePosesFromExpectedNeedlePose(expectedTipPose, "grasp", M_PI/8, samples_num_, new_samples);
      if (new_samples.empty())
          return false;

      // Check for collision-free trajectories in parallel
      #pragma omp parallel for
      for (size_t i = 0; i < new_samples.size(); i++)
      {
          float cost = 0;
          auto traj = getPathWithOmpl(T_B_Fee_, new_samples[i].pose, unit_id, "move", cost);

          if (traj.size() > 0)
          {
              #pragma omp critical
              {
                // ROS_WARN_STREAM("unit id " << unit_id);
                // ROS_WARN_STREAM("new_samples.size()   =" << new_samples.size());
                // ROS_WARN("can reach the pose ...?");
              //   T_B_Fee_cmd_ = new_samples[i].pose;
              // publishAsarEEPoseTarget();
              //   sleep(5.0);
                  // Early return if a valid trajectory is found by any thread
                  return true;
              }
          }
      }

      return false;
  }

  bool checkIfPoseCollisionFree(PoseAndJointValues candidatePose)
  {
    // ROS_INFO("coll-n function is called ...");
    asar_hybrid_tmp::collisionCheck srv;
    srv.request.arm_index = "unit" + std::to_string(unit_id);
    poseSE3ToMsg(candidatePose.pose, srv.request.checkPose);
    
    // ROS_INFO("before assigning the joint values ...");
    std::vector<float> jointValues(candidatePose.jointValues.size());
    for (int i = 0; i < candidatePose.jointValues.size(); ++i) {
        jointValues[i] = static_cast<float>(candidatePose.jointValues[i]);
        // ROS_INFO_STREAM(" i = " <<  i);
    }
    srv.request.jointValues = jointValues;
    // ROS_INFO("collision service request sent ...");
    collisionCheckerClient.call(srv);
    // ROS_WARN("colllision service response is received ...");
    return srv.response.collision_free;
  }

  void add_noise_to_grasp()
  {
    // changes the pose of the needle after the grasp is applied;
    asar_hybrid_tmp::AddNoise s;
    s.request.arm_id = "unit_" + std::to_string(unit_id);
    addNoiseClient_.call(s);
    ros::Rate r(10);
    r.sleep();
  }


};