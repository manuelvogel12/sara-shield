 	
#ifndef TESTNODE_H
#define TESTNODE_H

#include <xbot2/xbot2.h>
#include <ros/ros.h>

#include <vector>
#include "safety_shield/safety_shield.h"
#include "safety_shield/human_reach.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/verify.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "reach_lib.hpp"
#include "visualization_msgs/MarkerArray.h"
#include "custom_robot_msgs/PositionsHeadered.h"

using namespace XBot;
class TestNode : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;

    void on_start() override;

    void run() override;


private:
    
    //Eigen::VectorXd _q_start, _v_start,_v_new,_q_ref,_q_home;
    Eigen::VectorXd _q, _q_obs;
    safety_shield::SafetyShield _shield;
    std::vector<reach_lib::Point> _dummy_human_meas;
    int _iteration = 0;
    chrono::steady_clock::time_point _st_time;
    ros::Subscriber _human_joint_sub;
    ros::Publisher _human_marker_pub;
    
    void human_joint_callback(const custom_robot_msgs::PositionsHeaderedConstPtr& msg);
    //void human_joint_callback(const std_msgs::String& data);
    //double _homing_time;
    //double _fake_time;
    
};





#endif //TESTNODE_H

