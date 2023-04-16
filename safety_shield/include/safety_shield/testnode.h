 	
#ifndef TESTNODE_H
#define TESTNODE_H

#include <xbot2/xbot2.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>


#include <vector>
#include "safety_shield/safety_shield.h"
#include "safety_shield/human_reach.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/verify.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "reach_lib.hpp"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "custom_robot_msgs/PositionsHeadered.h"

using namespace XBot;
class TestNode : public ControlPlugin
{

public:

    TestNode(const Args& args);

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;

    void on_start() override;

    void run() override;


private:
    
    //Eigen::VectorXd _q_start, _v_start,_v_new,_q_ref,_q_home;
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    Eigen::VectorXd _q, _q_obs;
    safety_shield::SafetyShield _shield;
    std::vector<reach_lib::Point> _dummy_human_meas;
    int _iteration = 0;
    chrono::steady_clock::time_point _st_time;
    ros::Subscriber _human_joint_sub;
    ros::Subscriber _model_state_sub;
    ros::Publisher _human_marker_pub;
    ros::Publisher _robot_marker_pub;
    
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

    void humanJointCallback(const custom_robot_msgs::PositionsHeaderedConstPtr& msg);

    void createPoints(visualization_msgs::MarkerArray& markers, int nb_points_to_add, int shape_type, 
    int color_type);

    void createCapsules(visualization_msgs::MarkerArray& markers, const std::vector<std::vector<double>>& capsules);

    void createSphere(const geometry_msgs::Point& pos, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker);

    void createCylinder(const geometry_msgs::Point& p1, const geometry_msgs::Point p2, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker);
    //void human_joint_callback(const std_msgs::String& data);
    //double _homing_time;
    //double _fake_time;
    
};





#endif //TESTNODE_H

