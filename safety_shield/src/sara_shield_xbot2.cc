#include "safety_shield/sara_shield_xbot2.h"

#include <string>
#include <type_traits>
#include <iostream>
#include <fstream>
#include <cstdlib>


SaraShieldXbot2::SaraShieldXbot2(const Args& args)
  :ControlPlugin(args),
  _tfBuffer(tf2_ros::Buffer()),
  _tfListener(tf2_ros::TransformListener(_tfBuffer))
{
}

bool SaraShieldXbot2::on_initialize()
{
    //init ROS
    // ros: subscribe to topics and advertise topics
    ros::NodeHandle nh;
    _model_state_sub = nh.subscribe("/gazebo/model_states", 100, &SaraShieldXbot2::modelStatesCallback, this);
    _human_joint_sub = nh.subscribe("/human_pose_measurement", 100, &SaraShieldXbot2::humanJointCallback, this);
    _robot_goal_pos_sub = nh.subscribe("/sara_shield/goal_joint_pos", 100, & SaraShieldXbot2::goalJointPosCallback, this);
    _robot_trajectory_sub = nh.subscribe("/sara_shield/trajectory", 100, &SaraShieldXbot2::trajectoryCallback, this);
    _force_safe_sub = nh.subscribe("/sara_shield/force_safe", 100, & SaraShieldXbot2::forceSafeCallback, this);
    _force_unsafe_sub = nh.subscribe("/sara_shield/force_unsafe", 100, & SaraShieldXbot2::forceUnsafeCallback, this);
    _send_dummy_meas_sub = nh.subscribe("/sara_shield/send_dummy_meas", 100, &SaraShieldXbot2::sendDummyMeasFlagCallback, this);
    _humans_in_scene_sub = nh.subscribe("/sara_shield/humans_in_scene", 100, &SaraShieldXbot2::humansInSceneCallback, this);

    _human_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/sara_shield/human_joint_marker_array", 100);
    _robot_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/sara_shield/robot_joint_marker_array", 100);
    _robot_current_pos_pub = nh.advertise<std_msgs::Float32MultiArray>("/sara_shield/current_joint_pos", 100);
    _static_human_pub = nh.advertise<concert_msgs::Humans>("/human_pose_measurement", 100);
    _sara_shield_safe_pub = nh.advertise<std_msgs::Bool>("/sara_shield/is_safe", 100);
    
    
    // we must explicitly set the control mode for our robot
    // in this case, we will only send positions
    setDefaultControlMode(ControlMode::Position() + ControlMode::Velocity());
    // _robot->setControlMode(ControlMode::Position() + ControlMode::Velocity());

    bool activate_shield = true;

    double sample_time = getPeriodSec();

    std::string trajectory_config_file = getParamOrThrow<std::string>("~trajectory_config_file");
    std::string robot_config_file = getParamOrThrow<std::string>("~robot_config_file");
    std::string mocap_config_file = getParamOrThrow<std::string>("~mocap_config_file");

    double init_x = 0.0;
    double init_y = 0.0;
    double init_z = 0.0;
    double init_roll = 0.0;
    double init_pitch = 0.0;
    double init_yaw = 0.0;
    std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    _shield = safety_shield::SafetyShield(activate_shield,
      sample_time,
      trajectory_config_file,
      robot_config_file,
      mocap_config_file,
      init_x,
      init_y,
      init_z,
      init_roll,
      init_pitch,
      init_yaw,
      init_qpos,
      4);


    // Dummy human measurement
    _human_meas.resize(21);
    for (int i=0; i<21; i++) {
      _human_meas[i] = reach_lib::Point(400.0, 400.0, 0.0);
    }

    return true;

    //auto start_time = std::chrono::system_clock::now();
    //double t = std::chrono::duration<double>(std::chrono::system_clock::now()-start_time).count();
    //spdlog::info("Debug started.");
}


void SaraShieldXbot2::on_start()
{
    _robot->sense();
    
    Eigen::VectorXd init_qpos_arm_eigen;
    _robot->chain("chain_E").getJointPosition(init_qpos_arm_eigen);

    std::cout << "starting from q = " << init_qpos_arm_eigen.transpose() << "\n";

    std::vector<double> init_qpos_arm(init_qpos_arm_eigen.data(),
                                      init_qpos_arm_eigen.data() + init_qpos_arm_eigen.size());

    _shield.reset(true, 0, 0, 0, 0, 0, 0, init_qpos_arm, ros::Time::now().toSec());

}


void SaraShieldXbot2::run()
{
    _iteration++;

    // publish dummy humans
    if (_send_dummy_measurement_flag) {
      sendDemoHuman();
    }

    // check if a new goal pose is set. If so, give a new LongTermTrajectory to sara shield
    if(_new_goal){
      _new_goal = false;
       std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      _shield.newLongTermTrajectory(_goal_joint_pos, qvel);
    }

    //Perform a sara shield update step
    safety_shield::Motion next_motion = _shield.step(ros::Time::now().toSec());
    std::vector<double> q = next_motion.getAngle();

    //transform float array to Eigen Vector
    Eigen::Map<Eigen::VectorXd> q_eigen(&q[0], q.size()); 

    // Move the robot
    _robot->chain("chain_E").setPositionReference(q_eigen);
    _robot->move();


    // Visualize in every n-th timestep (using Ros visualization markers)
    if (_iteration % 5 == 0) {
      visualizeRobotAndHuman();
    }

    bool debug = true;
    if(debug && _iteration % 5 == 0) {
      //print angles of the robot
      //std::cout<<"q at time t=" << ros::Time::now().toSec() << ": ";
      //for(double d: q){
      //    std::cout<<std::fixed<<std::setprecision(3)<<d<<",";
      //}
      //std::cout<<std::endl;


      if(!_shield.getSafety()){
        jerror("NOT SAFE");
      }
      //else{
      //  jwarn("SAFE");
      //}

      //print the time the update step took
      //double time_after = std::chrono::duration<double>(std::chrono::system_clock::now()-time_before).count();
      //std::cout<<"shield step "<<_iteration<<" took:"<<time_after<<" seconds"<<std::endl;
    }

    // Send status bool
    std_msgs::Bool status;
    status.data = _shield.getSafety();
    _sara_shield_safe_pub.publish(status);

    //Send current pos message
    Eigen::VectorXd qpos_obs_arm_eigen;
    std_msgs::Float32MultiArray current_state_msg;
    _robot->sense();
    _robot->chain("chain_E").getJointPosition(qpos_obs_arm_eigen);
    std::vector<float> qpos_obs_float_vec(qpos_obs_arm_eigen.data(), 
                                          qpos_obs_arm_eigen.data()+qpos_obs_arm_eigen.size());
    current_state_msg.data = qpos_obs_float_vec;
    _robot_current_pos_pub.publish(current_state_msg);

    ros::spinOnce();
}

//convert the gazebo transformation between world and base_link into a tf
void SaraShieldXbot2::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  // find the index of the robot transform in the list of transforms
  int index = -1;
  for (uint64_t i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "ModularBot") {
      index = i;
      break;
    }
  }

  if (index == -1) {
    ROS_ERROR("Could not find robot modular_robot in model states message");
    return;
  }

  // make a tf transform out of the given gazebo transform
  tf::Transform transform;
  geometry_msgs::Pose pose = msg->pose[index];
  transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

  //publish the transform
  static tf::TransformBroadcaster br;
  ROS_DEBUG("Incoming Transform");
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

void SaraShieldXbot2::visualizeRobotAndHuman(){
  // visualize the robot and human
  visualization_msgs::MarkerArray humanMarkerArray = visualization_msgs::MarkerArray();
  visualization_msgs::MarkerArray robotMarkerArray = visualization_msgs::MarkerArray();

  // visualization of the human capsules
  std::vector<std::vector<double>> humanCapsules =_shield.getHumanReachCapsules(1);
  createPoints(humanMarkerArray, 3 * humanCapsules.size(),
               visualization_msgs::Marker::CYLINDER, 2);
  createCapsules(humanMarkerArray, humanCapsules);

  // visualization of the robot capsules
  std::vector<std::vector<double>> robotReachCapsules =_shield.getRobotReachCapsules();
  createPoints(robotMarkerArray, 3 * robotReachCapsules.size(),
               visualization_msgs::Marker::CYLINDER, _shield.getSafety()?0:1);
  createCapsules(robotMarkerArray, robotReachCapsules);

  _human_marker_pub.publish(humanMarkerArray);
  _robot_marker_pub.publish(robotMarkerArray);

}


// Reads the human pose from the Gazebo msg, and uses it for sara_shield
void SaraShieldXbot2::humanJointCallback(const concert_msgs::HumansConstPtr& msg) {
  // get the robot position
  geometry_msgs::TransformStamped transformation;
  std::string source_frame =  msg->header.frame_id;
  if(source_frame == ""){
    source_frame="base_link";
  }

  try {
    transformation = _tfBuffer.lookupTransform(
        "base_link", source_frame, msg->header.stamp, ros::Duration(0.003));
  } catch (tf2::LookupException const&) {
    ROS_WARN("NO TRANSFORM FOUND (Lookup failed)");
    return;
  } catch (tf2::ExtrapolationException const&) {
    ROS_WARN("NO TRANSFORM FOUND (ExtrapolationException)");
    return;
  }

  //get all human measurement points and transform them to the robot coordinate system
  if(msg->humans.size() > 0)
  {
    for (const concert_msgs::Human3D &human: msg->humans){
      _human_meas.clear();
      int human_index = human.label_id;
      for(const concert_msgs::Keypoint3D &keypoint:human.keypoints)
      {
        geometry_msgs::PointStamped pointStamped;
        geometry_msgs::PointStamped pointStampedLocal;
        pointStamped.point = keypoint.pose.position;    
        tf2::doTransform(pointStamped, pointStampedLocal, transformation);
        geometry_msgs::Point pointLocal = pointStampedLocal.point;
        
        _human_meas.emplace_back(
            reach_lib::Point(pointLocal.x, pointLocal.y, pointLocal.z));
      }
      double messageTime = msg->header.stamp.toSec();
      if(messageTime == 0.0){
        messageTime = ros::Time::now().toSec();
      }
      _shield.humanMeasurement(_human_meas, human_index, messageTime);

    }
  }
}


void SaraShieldXbot2::sendDemoHuman()
{
  concert_msgs::Human3D human;
  for(int i = 0; i < 25; i++){
    concert_msgs::Keypoint3D keyPoint;
    keyPoint.pose.position.x = 1.0;
    keyPoint.pose.position.y = 0.0;
    keyPoint.pose.position.z = 0.2;
    human.keypoints[i] = keyPoint;
  }
  // Pelv
  human.keypoints[24].pose.position.x = 1.0;
  human.keypoints[24].pose.position.y = 0.0;
  human.keypoints[24].pose.position.z = 0.3;

  // Collar
  human.keypoints[11].pose.position.x = 1.0;
  human.keypoints[11].pose.position.y = 0.0;
  human.keypoints[11].pose.position.z = 0.8;

  // Neck
  human.keypoints[12].pose.position.x = 1.0;
  human.keypoints[12].pose.position.y = 0.0;
  human.keypoints[12].pose.position.z = 1.0;

  // Head
  human.keypoints[13].pose.position.x = 1.0;
  human.keypoints[13].pose.position.y = 0.0;
  human.keypoints[13].pose.position.z = 1.2;

  // Left shoulder
  human.keypoints[14].pose.position.x = 1.01;
  human.keypoints[14].pose.position.y = 0.26;
  human.keypoints[14].pose.position.z = 0.95;

  // Right Shoulder
  human.keypoints[19].pose.position.x = 1.0;
  human.keypoints[19].pose.position.y = -0.26;
  human.keypoints[19].pose.position.z = 0.95;


  // Left elbow
  human.keypoints[15].pose.position.x = 1.0;
  human.keypoints[15].pose.position.y = 0.27;
  human.keypoints[15].pose.position.z = 0.65;

  // Right Elbow
  human.keypoints[20].pose.position.x = 1.0;
  human.keypoints[20].pose.position.y = -0.27;
  human.keypoints[20].pose.position.z = 0.65;

  // Left Wrist
  human.keypoints[16].pose.position.x = 1.0;
  human.keypoints[16].pose.position.y = 0.28;
  human.keypoints[16].pose.position.z = 0.3;


  // Right Wrist
  human.keypoints[21].pose.position.x = 1.0;
  human.keypoints[21].pose.position.y = -0.28;
  human.keypoints[21].pose.position.z = 0.3;

  // Left Hand
  human.keypoints[17].pose.position.x = 1.0;
  human.keypoints[17].pose.position.y = 0.29;
  human.keypoints[17].pose.position.z = 0.2;

  // Right Hand
  human.keypoints[22].pose.position.x = 1.0;
  human.keypoints[22].pose.position.y = -0.29;
  human.keypoints[22].pose.position.z = 0.2;

  concert_msgs::Humans humans;
  humans.humans.push_back(human);
  
  humans.header.frame_id = "base_link";
  humans.header.stamp = ros::Time::now();

  //std::cout<<"SEND HUMAN"<<std::endl;
  _static_human_pub.publish(humans);
}

void SaraShieldXbot2::goalJointPosCallback(const std_msgs::Float32MultiArray& msg)
{
  std::cout<<"RECEIVE POS"<<std::endl;
  std::vector<double> a(msg.data.begin(), msg.data.end());
  _goal_joint_pos = a;
  _new_goal = true;
}

void SaraShieldXbot2::trajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  std::cout<<"Receive Long Term Trajectory"<<std::endl;
  std::vector<safety_shield::Motion> long_term_traj = {};
  double init_time = _shield.getPath_S();
  double trajectory_sample_time = msg->points[1].time_from_start.toSec() - msg->points[0].time_from_start.toSec();
  // Debug only!
  double speed_factor = 1;
  trajectory_sample_time = trajectory_sample_time / speed_factor;
  for(trajectory_msgs::JointTrajectoryPoint traj_point: msg->points){
    double time = init_time + traj_point.time_from_start.toSec() / speed_factor;
    // TODO make it more general (check names of joints or something)

    const std::vector<double> q(traj_point.positions.end() - 6, traj_point.positions.end());
    const std::vector<double> dq = (traj_point.velocities.size() >= 6) ?
                                    std::vector<double>(traj_point.velocities.end() - 6, traj_point.velocities.end()) :
                                    std::vector<double>(traj_point.velocities.begin(), traj_point.velocities.end());
    const std::vector<double> ddq = (traj_point.accelerations.size() >= 6) ?
                                  std::vector<double>(traj_point.accelerations.end() - 6, traj_point.accelerations.end()) :
                                  std::vector<double>(traj_point.accelerations.begin(), traj_point.accelerations.end());
    
    
    // create Motion depending on which properties are provided
    if (ddq.size() >= 6 && dq.size() >= 6)
    {
      long_term_traj.emplace_back(time, q, dq, ddq);  
    }else if (dq.size() >= 6)
    {
      long_term_traj.emplace_back(time, q, dq);
    }else
    {
      long_term_traj.emplace_back(time, q);
    }
  }
  int start_index = static_cast<int>(ceil(init_time / trajectory_sample_time));
  int sliding_window_k = _shield.getSliding_window_k();
  safety_shield::LongTermTraj traj = safety_shield::LongTermTraj(long_term_traj, trajectory_sample_time, start_index, sliding_window_k);
  std::cout<<"Set Long Term Trajectory"<<std::endl;
  _shield.setLongTermTrajectory(traj);
}


void SaraShieldXbot2::forceSafeCallback(const std_msgs::Bool & msg){
  _shield.setForceSafe(msg.data);
}

void SaraShieldXbot2::forceUnsafeCallback(const std_msgs::Bool & msg){
  _shield.setForceUnsafe(msg.data);
}

void SaraShieldXbot2::sendDummyMeasFlagCallback(const std_msgs::Bool& msg) {
  _send_dummy_measurement_flag = msg.data;
}


void SaraShieldXbot2::humansInSceneCallback(const std_msgs::Bool& msg) {
  if (!msg.data) {
    _shield.noHumanInTheScene();
  }
}


void SaraShieldXbot2::createPoints(visualization_msgs::MarkerArray& markers, int nb_points_to_add, int shape_type, 
    int color_type) {
  int prev_size = markers.markers.size();
  for(int i = 0; i < nb_points_to_add; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id="base_link";
    marker.ns = "shapes";
    marker.id = prev_size+i;
    marker.type = shape_type;
    if(color_type == 0) { // ROBOT (safe)
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    }
    else if (color_type == 1) { // ROBOT (unsafe)
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
    } else if (color_type == 2) { //HUMAN_REACH
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
    }   
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    markers.markers.push_back(marker);
  }
}

void SaraShieldXbot2::createCapsules(visualization_msgs::MarkerArray& markers, const std::vector<std::vector<double>>& capsules) { 
  auto marker = markers.markers.begin();
  for(const std::vector<double>& cap :capsules) {
    geometry_msgs::Point p1;
    p1.x = cap[0];
    p1.y = cap[1];
    p1.z = cap[2];
    geometry_msgs::Point p2;
    p2.x = cap[3];
    p2.y = cap[4];
    p2.z = cap[5];
    double radius = cap[6];
    //first circle
    createSphere(p1, radius, ros::Time::now(), *marker);
    marker++;
    //second circle
    createSphere(p2, radius, ros::Time::now(), *marker);
    marker++;
    //middle cylinder
    createCylinder(p1, p2, radius, ros::Time::now(), *marker);
    marker++;
  }
}

void SaraShieldXbot2::createSphere(const geometry_msgs::Point& pos, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker) {
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position = pos;
  marker.scale.x = 2*radius;
  marker.scale.y = 2*radius;
  marker.scale.z = 2*radius;
  marker.header.stamp = stamp;
}


void SaraShieldXbot2::createCylinder(const geometry_msgs::Point& p1, const geometry_msgs::Point p2, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker) {
  double p1x = p1.x;
  double p1y = p1.y;
  double p1z = p1.z;
  double p2x = p2.x;
  double p2y = p2.y;
  double p2z = p2.z;
  double v2_x = (p2x-p1x);
  double v2_y = (p2y-p1y);
  double v2_z = (p2z-p1z);
  double norm = sqrt(pow(v2_x, 2) + pow(v2_y, 2) + pow(v2_z, 2));
  if(norm > 1e-6) {
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = (p1x + p2x)/2;
    marker.pose.position.y = (p1y + p2y)/2;
    marker.pose.position.z = (p1z + p2z)/2;
    // Rotate z axis vector to direction vector according to https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another/1171995#1171995
    double a_x = -v2_y/norm;
    double a_y = v2_x/norm;
    double a_z = 0;
    double a_w = 1 + v2_z/norm;
    double norm_q = sqrt(pow(a_w, 2) + pow(a_x, 2) + pow(a_y, 2) + pow(a_z, 2));
    marker.pose.orientation.w = a_w/norm_q;
    marker.pose.orientation.x = a_x/norm_q;
    marker.pose.orientation.y = a_y/norm_q;
    marker.pose.orientation.z = a_z/norm_q;
    marker.scale.z = norm;
    marker.scale.y = 2*radius;
    marker.scale.x = 2*radius;
  }
  else{
    marker.scale.x = 0;
    marker.scale.y = 0;
    marker.scale.z = 0;
  }
  marker.header.stamp = stamp;
}

XBOT2_REGISTER_PLUGIN(SaraShieldXbot2, sara_shield_xbot2)

