// -*- lsst-c++ -*/
/**
 * @file testnode.h
 * @brief execute the sara shield in the xbot-concert framework
 * @version 0.1
 * @copyright This file is part of SaRA-Shield.
 * SaRA-Shield is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 * SaRA-Shield is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with SaRA-Shield. 
 * If not, see <https://www.gnu.org/licenses/>. 
 */

#ifndef TESTNODE_H
#define TESTNODE_H

#include "reach_lib.hpp"
#include "safety_shield/safety_shield.h"
#include "safety_shield/human_reach.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/verify.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"

#include <xbot2/xbot2.h>
#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "custom_robot_msgs/PositionsHeadered.h"

#include "custom_robot_msgs/Humans.h"

#include <vector>

using namespace XBot;
/**
 * @brief Node to execute sara shield in the xbot-concert framework
 */
class TestNode : public ControlPlugin
{

public:

    /**
     * @brief  Constructor to initialize tf2 Buffer and tf2 Listener
     * @param  args: arguments for the parent(XBot::ControlPlugin) constructor
     */
    TestNode(const Args& args);

    //using ControlPlugin::ControlPlugin;

    /**
     * @brief Initialize the plugin
     */
    bool on_initialize() override;

    /**
     * @brief Starting the plugin (mostly initializing sara shield)
     */
    void on_start() override;

    /**
     * @brief  called in every iteration, performing a single time step of sara shield
     * @note   
     * @retval None
     */
    void run() override;


private:
    
    // Eigen::VectorXd _q_start, _v_start,_v_new,_q_ref,_q_home;
    
   /**
    * @brief DEBUG, vectors to get/set the robot position   
    * @note mostly for testing, should be replaced by local variables
    */
    Eigen::VectorXd _q, _q_obs;

    /**
     * @brief the sara shield
     */
    safety_shield::SafetyShield _shield;

    /**
     * @brief Human measurement points to be used in sara shield
     */
    std::vector<reach_lib::Point> _human_meas;

   /**
    * @brief DEBUG, numbering the time steps
    */
    int _iteration = 0;
   
   /**
    * @brief DEBUG, starting time of plugin
    */
    // chrono::steady_clock::time_point _st_time;

    std::vector<double> _goal_joint_pos;
    
    bool _new_goal = false;
    // ROS

    /**
     * @brief tf2 transformation buffer to lookup the position and orientation of the robot
     */
    tf2_ros::Buffer _tfBuffer;
       
    /**
     * @brief tf2 Listener to get transforms
     */
    tf2_ros::TransformListener _tfListener;

    /**
     * @brief Ros subsriber to receveive human joint positions
     */
    ros::Subscriber _human_joint_sub;
    
    /**
     * @brief Ros subscriber to receive Robot position and orientation
     */
    ros::Subscriber _model_state_sub;
    
    ros::Subscriber _robot_goal_pos_sub;

    ros::Subscriber _safe_flag_sub;

    /**
     * @brief Ros publisher to publish human position visualization
     */
    ros::Publisher _human_marker_pub;
    
    /**
     * @brief Ros publisher to publish robot visualization
     */
    ros::Publisher _robot_marker_pub;

    ros::Publisher _static_human_pub;   
    
    /**
     * @brief Convert the gazebo transformation between world and base_link into a tf
     * @param msg the gazebo model state msg
     */
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

    /**
     * @brief Reads the human pose from the Gazebo msg, and uses it for sara_shield. Also publishes visualization msgs of the human meas points
     * @param msg human pose msg
     */
    void humanJointCallback(const custom_robot_msgs::HumansConstPtr& msg);

    /**
     * @brief create points for the visualization of the robot and human in rviz
     * @param[out] markers the created visualization markers 
     * @param nb_points_to_add number of points to visualize
     * @param shape_type The marker shape (e.g. cylinder)
     * @param color_type Color of the points (0: robot, 1: human cylinder, 2: human reach)
     */
    void createPoints(visualization_msgs::MarkerArray& markers, int nb_points_to_add, int shape_type, 
    int color_type);

    /**
     * @brief Create a capsule for visualization, consisting of one cylinder and two spheres 
     * @param[out] markers resulting array of visualization markers  
     * @param capsules vector of capsule descriptions
     */
    void createCapsules(visualization_msgs::MarkerArray& markers, const std::vector<std::vector<double>>& capsules);

    /**
     * @brief create a visualization sphere
     * @param pos center point of the sphere
     * @param radius radius of the sphere
     * @param stamp timestamp
     * @param[out] marker output marker
     */
    void createSphere(const geometry_msgs::Point& pos, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker);

    /**
     * @brief create a visualization cylinder
     * @param p1 endpoint 1
     * @param p2 end point 2
     * @param radius thickness of the cylinder
     * @param stamp timestamp
     * @param[out] marker output marker
     */
    void createCylinder(const geometry_msgs::Point& p1, const geometry_msgs::Point p2, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker);


    void visualizeRobotAndHuman();  

    void goalJointPosCallback(const std_msgs::Float32MultiArray& msg);

    void safeFlagCallback(const std_msgs::Bool & msg);

    void sendDemoHuman();
};

#endif //TESTNODE_H
