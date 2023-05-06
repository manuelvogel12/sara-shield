#include "safety_shield/testnode.h"

#include <string>
#include <vector>
#include <chrono>
#include <type_traits>
#include <iostream>
#include <fstream>
#include "fmt/chrono.h"
#include <cstdlib>


TestNode::TestNode(const Args& args)
  :ControlPlugin(args),
  _tfBuffer(tf2_ros::Buffer()),
  _tfListener(tf2_ros::TransformListener(_tfBuffer))
{
}

bool TestNode::on_initialize()
{
    //init ROS
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "testnode");
    return true;
}



void TestNode::on_start()
{
    // ros: subsribe to topics and advertise topics
    ros::NodeHandle nh;
    _model_state_sub = nh.subscribe("/gazebo/model_states", 100, &TestNode::modelStatesCallback, this);
    _human_joint_sub = nh.subscribe("/human_joint_pos", 100, &TestNode::humanJointCallback, this);
    _human_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/human_joint_marker_array", 100);
    _robot_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/robot_joint_marker_array", 100);
    
    // we must explicitly set the control mode for our robot
    // in this case, we will only send positions
    _robot->setControlMode(ControlMode::Position() + ControlMode::Velocity());
    
    // do some on-start initialization
    _robot->sense();
    //_robot->getPositionReference(_q_start);
    //_robot->getVelocityReference(_v_start);

    bool activate_shield = true;
    _st_time = chrono::steady_clock::now();

    double sample_time = 0.001;
    //Note: executing directory is /tmp/something/ TODO: make path relative somehow
    //std::string trajectory_config_file = std::string("/home/user/concert_ws/src/sara-shield/safety_shield/config/trajectory_parameters_schunk.yaml");
    //std::string robot_config_file = std::string("/home/user/concert_ws/src/sara-shield/safety_shield/config/robot_parameters_schunk.yaml");
    //std::string mocap_config_file = std::string("/home/user/concert_ws/src/sara-shield/safety_shield/config/cmu_mocap_no_hand.yaml");
    std::string trajectory_config_file = std::string(std::getenv("HOME")) + "/concert_ws/src/sara-shield/safety_shield/config/trajectory_parameters_schunk.yaml";
    std::string robot_config_file = std::string(std::getenv("HOME")) + "/concert_ws/src/sara-shield/safety_shield/config/robot_parameters_schunk.yaml";
    std::string mocap_config_file = std::string(std::getenv("HOME")) + "/concert_ws/src/sara-shield/safety_shield/config/cmu_mocap_no_hand.yaml";
    //std::string robot_config_file = std::string("/tmp/robot_parameters_schunk.yaml");

    double init_x = 0.0;
    double init_y = 0.0;
    double init_z = 0.0;
    double init_roll = 0.0;
    double init_pitch = 0.0;
    double init_yaw = 0.0;
    std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //YAML::Node robot_config = YAML::LoadFile(robot_config_file);
    //std::cout<<robot_config["robot_name"].as<std::string>()<<std::endl;

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
      init_qpos);

    // Dummy human measurement

    _dummy_human_meas.resize(21);
    for (int i=0; i<21; i++) {
      _dummy_human_meas[i] = reach_lib::Point(400.0, 400.0, 0.0);
    }


    //auto start_time = std::chrono::system_clock::now();
    //double t = std::chrono::duration<double>(std::chrono::system_clock::now()-start_time).count();
    //spdlog::info("Debug started.");

    // we can switch to run
    start_completed();

}

void TestNode::run()
{

    _iteration++;

    auto st_elapsed = chrono::steady_clock::now() - _st_time;
    double t = std::chrono::duration<double>(st_elapsed).count();

    //if(t > 5){
    //	std::vector<reach_lib::Point> measures = {reach_lib::Point(0.0, 0.0, 0.0)};
    //	_shield.humanMeasurement(measures, t) ;
    //}else{
        _shield.humanMeasurement(_dummy_human_meas, t);
    //}
    

    st_elapsed = chrono::steady_clock::now() - _st_time;
    t = std::chrono::duration<double>(st_elapsed).count();

    //Dummy movement

    // if (_iteration % 5 == 0) {
    //     std::vector<double> qpos{0.2*t, -0.1*t, -0.1*t, 0.0, -0.1*t, 0.0, 0.0, -0.1*t, 0.0, 0.0, 0.0, 0.0, std::min(t, 3.1)};
    //     std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //     _shield.newLongTermTrajectory(qpos, qvel);
    // }
    if (_iteration % 10 == 1) {   //Joint order: 0:Unknown 1:J1_EE 2:J2_EE  3:J3_EE  4:J4_EE  5:J5_EE 6: ... 
        std::vector<double> qpos{0.02*t, 0.02*t, -0.02*t, -0.02*t, -0.02*t, 0.02*t, 0.02*t, 0.02*t, 0.02*t, 0.02*t, 0.02*t, 0.02*t, 0.02*t};
        std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        _shield.newLongTermTrajectory(qpos, qvel);
    }

    //debug: measure performance
    auto time_before = std::chrono::system_clock::now();
    safety_shield::Motion next_motion = _shield.step(t);
    if(!_shield.getSafety()){
      jerror("NOT SAFE");
    }
    //else{
    //  jwarn("SAFE");
    //}
    double time_after = std::chrono::duration<double>(std::chrono::system_clock::now()-time_before).count();
    std::cout<<"shield step "<<_iteration<<" took:"<<time_after<<" seconds"<<std::endl;

    //std::cout<< "Angles"<< std::endl;
    //std::vector<double> q = next_motion.getAngle();
    //for(double& angle: q)
    //    std::cout << angle << std::endl;
    //std::cout<< "Velo"<< std::endl;
    //std::vector<double> v = next_motion.getVelocity();
    //for(double& velocity: v)
    //    std::cout << velocity << std::endl;

    std::vector<double> q = next_motion.getAngle();
    //change size of q to 13
    for(int i = q.size(); i< 13; i++)
    	q.insert(q.begin(), 0);

    //TODO: remove DEBUG
    std::cout<<"q at time t="<<t<<": ";
    for(double d: q){
        std::cout<<d<<",";
    }
    std::cout<<std::endl;

    //Eigen::VectorXd _q = Eigen::VectorXd::Zero(13);
    Eigen::Map<Eigen::VectorXd> _q(&q[0], q.size()); 
    //_q *= 1.0;
    _robot->setPositionReference(_q);
    _robot->move();

    _robot->getPositionReference(_q_obs);
    //std::cout<<"observed: \n "<<_q_obs<<std::endl;
    //std::cout<<"safe:" <<_shield.getSafety()<<std::endl;


    //double t2 = std::chrono::duration<double>(elapsed_2).count();
    //double t3 = std::chrono::duration<double>(elapsed_3).count();
    //double t4 = std::chrono::duration<double>(elapsed_4).count();
    //double t5 = std::chrono::duration<double>(elapsed_5).count();
    //double t6 = std::chrono::duration<double>(elapsed_6).count();
    //double t7 = std::chrono::duration<double>(elapsed_7).count();
    //std::cout<<"2: "<<t2<<" |3: "<<t3<<" |4: "<<t4<<" |5: "<<t5	<<" |6: "<<t6<<" |7: "<<t7<<std::endl;		

    //_shield.reset(true, init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t);
      
    
    // define a simplistic linear trajectory
    //double tau = _fake_time/_homing_time;

    // quintic poly 6t^5 - 15t^4 + 10t^3
    //double alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;

    // interpolate
    //_q_ref = _q_start + alpha * (_q_home - _q_start);

    
    //_q_offset << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    //Eigen::VectorXd a2 = Eigen::VectorXd::Random(13);
    //a2 *= 0.0001; 
    //a2 += _q_start;
    //jerror("Current Position '{}' --- '{}' \n",_q_start, a2);
    //_v_start << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    //_v_start = _v_start + _v_new;

   // _robot->sense();
    //_robot->getPositionReference(_q_start);
   
    
    // send reference
    //_robot->setPositionReference(a2);
    
    //_robot->move();
    /*
    // if trajectory ended, we stop ourselves
    //if(tau >= 1.0)
    //{
    //    stop();
    //    return;
    //}

    // increment fake time
    // note: getPeriodSec() returns the nominal period
    //_fake_time += getPeriodSec();
    */
    ros::spinOnce();
}

//convert the gazebo transformation between world and base_link into a tf
void TestNode::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
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

  tf::Transform transform;
  geometry_msgs::Pose pose = msg->pose[index];
  transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

  static tf::TransformBroadcaster br;
  ROS_INFO("INCOMING TRANSFORM");
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}


// Reads the human pose from the Gazebo msg, and uses it for sara_shield. Also publishes visualization msgs of the human meas points
void TestNode::humanJointCallback(const custom_robot_msgs::PositionsHeaderedConstPtr& msg) {
  // get the robot position
  geometry_msgs::TransformStamped transformation;
  try {
    transformation = _tfBuffer.lookupTransform(
        "base_link", "world", msg->header.stamp, ros::Duration(0.003));
  } catch (tf2::LookupException const&) {
    ROS_WARN("NO TRANSFORM FOUND (Lookup failed)");
    return;
  } catch (tf2::ExtrapolationException const&) {
    ROS_WARN("NO TRANSFORM FOUND (ExtrapolationException)");
    return;
  }

  // visualize the robot and human
  visualization_msgs::MarkerArray humanMarkerArray = visualization_msgs::MarkerArray();
  visualization_msgs::MarkerArray robotMarkerArray = visualization_msgs::MarkerArray();
  _dummy_human_meas.clear();
  
  //get all human measurment points and transform them to robot coordinate system
  for(const geometry_msgs::Point &point: msg->data) {

    geometry_msgs::PointStamped pointStamped;
    geometry_msgs::PointStamped pointStampedLocal;
    pointStamped.point = point;    
    tf2::doTransform(pointStamped,pointStampedLocal, transformation);
    geometry_msgs::Point pointLocal = pointStampedLocal.point;
    _dummy_human_meas.emplace_back(
        reach_lib::Point(pointLocal.x, pointLocal.y, pointLocal.z));
  }

  // visualization of Robot and Human Capsules
  std::vector<std::vector<double>> humanCapsules =
      _shield.getHumanReachCapsules(1);
  createPoints(humanMarkerArray, 3 * humanCapsules.size(),
               visualization_msgs::Marker::CYLINDER, 2);
  createCapsules(humanMarkerArray, humanCapsules);

  std::vector<std::vector<double>> robotReachCapsules =
      _shield.getRobotReachCapsules();
  createPoints(robotMarkerArray, 3 * robotReachCapsules.size(),
               visualization_msgs::Marker::CYLINDER, 0);
  createCapsules(robotMarkerArray, robotReachCapsules);

  _human_marker_pub.publish(humanMarkerArray);
  _robot_marker_pub.publish(robotMarkerArray);
}

void TestNode::createPoints(visualization_msgs::MarkerArray& markers, int nb_points_to_add, int shape_type, 
    int color_type) {
  int prev_size = markers.markers.size();
  for(int i = 0; i < nb_points_to_add; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id="base_link";
    marker.ns = "shapes";
    marker.id = prev_size+i;
    marker.type = shape_type;
    if(color_type == 0) { // ROBOT
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    }
    else if (color_type == 1) { // HUMAN_CYLINDER
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
    } else if (color_type == 2) { //HUMAN_REACH
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
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

void TestNode::createCapsules(visualization_msgs::MarkerArray& markers, const std::vector<std::vector<double>>& capsules) { 
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

void TestNode::createSphere(const geometry_msgs::Point& pos, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker) {
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position = pos;
  marker.scale.x = 2*radius;
  marker.scale.y = 2*radius;
  marker.scale.z = 2*radius;
  marker.header.stamp = stamp;
}


void TestNode::createCylinder(const geometry_msgs::Point& p1, const geometry_msgs::Point p2, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker) {
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


XBOT2_REGISTER_PLUGIN(TestNode, testnode)

