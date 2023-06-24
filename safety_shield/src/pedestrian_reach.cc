#include "safety_shield/pedestrian_reach.h"

namespace safety_shield {

PedestrianReach::PedestrianReach(
      //std::map<std::string, reach_lib::jointPair>& body_link_joints, 
      //const std::map<std::string, double>& thickness, 
      double max_v, 
      double max_a,
      double measurement_error_pos, 
      double measurement_error_vel, 
      double delay):
  //body_link_joints_(body_link_joints),
  measurement_error_pos_(measurement_error_pos),
  measurement_error_vel_(measurement_error_vel),
  delay_(delay)
{
  reach_lib::System system(measurement_error_pos, measurement_error_vel, delay);

  double height = 2.0;
  double arm_span = 1.0;
  pedestrian_p_ = reach_lib::Pedestrian(system, height, arm_span);
  pedestrian_v_ = reach_lib::PedestrianVel(system, height, arm_span, max_v);
  pedestrian_a_ = reach_lib::PedestrianAccel(system, height, arm_span, max_a);

  center_pos_ = reach_lib::Point(0.0, 0.0, 0.0);
  center_vel_ = reach_lib::Point(0.0, 0.0, 0.0);
}

void PedestrianReach::reset() {
  last_meas_timestep_ = -1;
  center_pos_ = reach_lib::Point(0.0, 0.0, 0.0);
  center_vel_ = reach_lib::Point(0.0, 0.0, 0.0);
}

void PedestrianReach::measurement(const reach_lib::Point& pedestrian_center_pos, double time) {
  try {
    if (last_meas_timestep_ != -1) {
      double dt = time - last_meas_timestep_;
      // If more than 1 measurement, calculate velocity
      center_vel_ = (pedestrian_center_pos - center_pos_) * (1/dt);
      has_second_meas_ = true;
    }
    center_pos_ = pedestrian_center_pos;
    last_meas_timestep_ = time;
    //ROS_INFO_STREAM("Human Mocap measurement received. Timestamp of meas was " << last_meas_timestep);
  } catch (const std::exception &exc) {
    //spdlog::error("Exception in HumanReach::measurement: {}", exc.what());
  }
}


void PedestrianReach::humanReachabilityAnalysis(double t_command, double t_brake) {
  try {
    // Time between reach command msg and last measurement plus the t_brake time.
    double t_reach = t_command-last_meas_timestep_ + t_brake;
    // Calculate reachable set

    //TODO: fix
    std::vector<reach_lib::Point> joint_pos = {center_pos_};
    std::vector<reach_lib::Point> joint_vel = {center_vel_};

    pedestrian_p_.update(0.0, t_reach, joint_pos, joint_vel);
    pedestrian_v_.update(0.0, t_reach, joint_pos, joint_vel);
    pedestrian_a_.update(0.0, t_reach, joint_pos, joint_vel);
  } catch (const std::exception &exc) {
      //spdlog::error("Exception in HumanReach::humanReachabilityAnalysis: {}", exc.what());
  }
}

} // namespace safety_shield


