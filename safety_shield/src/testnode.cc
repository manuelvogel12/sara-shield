#include "safety_shield/testnode.h"

#include <string>
#include <vector>
#include <chrono>
#include <type_traits>
#include <iostream>
#include <fstream>
#include "fmt/chrono.h"

bool TestNode::on_initialize()
{
    return true;
}



void TestNode::on_start()
{
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
    std::string trajectory_config_file = std::string("/home/user/concert_ws/src/sara-shield/safety_shield/config/trajectory_parameters_schunk.yaml");
    std::string robot_config_file = std::string("/home/user/concert_ws/src/sara-shield/safety_shield/config/robot_parameters_schunk.yaml");
    std::string mocap_config_file = std::string("/home/user/concert_ws/src/sara-shield/safety_shield/config/cmu_mocap_no_hand.yaml");
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
      _dummy_human_meas[i] = reach_lib::Point(10.0, 10.0, 0.0);
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

    _shield.humanMeasurement(_dummy_human_meas, t);

    st_elapsed = chrono::steady_clock::now() - _st_time;
    t = std::chrono::duration<double>(st_elapsed).count();

    if (_iteration % 2 == 0) {
        std::vector<double> qpos{0.2*t, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, std::min(t, 3.1)};
        std::vector<double> qvel{0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        _shield.newLongTermTrajectory(qpos, qvel);
    }

    safety_shield::Motion next_motion = _shield.step(t);

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
        q.push_back(0);

    //TODO: remove DEBUG
    std::cout<<"q at time t="<<t<<": ";
    for(double d: q){
        std::cout<<d<<",";
    }
    std::cout<<std::endl;

    //Eigen::VectorXd _q = Eigen::VectorXd::Zero(13);
    Eigen::Map<Eigen::VectorXd> _q(&q[0], q.size()); 
    _robot->setPositionReference(_q);
    _robot->move();
    
    _robot->getPositionReference(_q_obs);
    std::cout<<"observed: \n "<<_q_obs<<std::endl;


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

}

XBOT2_REGISTER_PLUGIN(TestNode, testnode)

