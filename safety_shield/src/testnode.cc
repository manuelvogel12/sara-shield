#include "safety_shield/testnode.h"

#include <string>
#include <vector>
#include <chrono>
#include <type_traits>
#include <iostream>






bool TestNode::on_initialize()
{

    // to make this plugin minimally configurable,
    // we take the homing state name from two internal
    // XBot2 parameters named '~home' and '~qhome'

    // we must explicitly set the control mode for our robot
    // in this case, we will only send positions
    _robot->setControlMode(ControlMode::Position() + ControlMode::Velocity());
    //set_control_mode();

    return true;
}


void TestNode::set_control_mode()
{
    std::map<std::string, ControlMode> ctrl_map;
    
    for(auto j : _robot->getEnabledJointNames())
    {
        ControlMode ctrl;
        if(!getParam("/hal/joint/control_mode/" + j, ctrl))
        {
            continue;
        }
        std::cout <<"control" <<ctrl.getName ()  << std::endl;
        
        
        ctrl_map[j] = ctrl;
    }
    
    _robot->setControlMode(ctrl_map);
}


void TestNode::starting()
{
    // do some on-start initialization
    _robot->sense();
    _robot->getPositionReference(_q_start);
    _robot->getVelocityReference(_v_start);
    //jerror("START  \n");
    
    
    //_shield = safety_shield::SafetyShield();
/*  
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
  */      
    // Dummy human measurement
    
    //for (int i=0; i<21; i++) {
    //  _dummy_human_meas[i] = reach_lib::Point(10.0, 10.0, 0.0);
    //}



    //auto start_time = std::chrono::system_clock::now();
    //double t = std::chrono::duration<double>(std::chrono::system_clock::now()-start_time).count();
    //spdlog::info("Debug started.");
    double t = 0.0;


    // we can switch to run
    start_completed();
}

void TestNode::run()
{
    
    
    bool activate_shield = true;
    double sample_time = 0.001; 
    //std::string trajectory_config_file = std::string("../config/trajectory_parameters_schunk.yaml");
    //std::string robot_config_file = std::string("../config/robot_parameters_schunk.yaml");
    //std::string mocap_config_file = std::string("../config/cmu_mocap_no_hand.yaml");
    std::string trajectory_config_file = std::string("/home/user/concert_ws/src/sara-shield/safety_shield/config/trajectory_parameters_schunk.yaml");
    std::string robot_config_file = std::string("/home/user/concert_ws/src/sara-shield/safety_shield/config/robot_parameters_schunk.yaml");
    std::string mocap_config_file = std::string("/home/user/concert_ws/src/sara-shield/safety_shield/config/cmu_mocap_no_hand.yaml");
    
    double init_x = 0.0;
    double init_y = 0.0;
    double init_z = 0.0;
    double init_roll = 0.0;
    double init_pitch = 0.0;
    double init_yaw = 0.0;
    std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //safety_shield::VerifyISO verif= safety_shield::VerifyISO();
        /*
        _iteration++;
	t += 0.001;
	_shield.humanMeasurement(_dummy_human_meas, t);
	t += 0.003;
	if (_iteration % 2 == 0) {
	    std::vector<double> qpos{0.2*t, 0.0, 0.0, 0.0, 0.0, std::min(t, 3.1)};
	    std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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
	//_robot->setPositionReference(q);
	//_robot->move();
      //_shield.reset(true, init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t);
      */
      
    jerror("NEW  \n");
    
    
    // define a simplistic linear trajectory
    //double tau = _fake_time/_homing_time;

    // quintic poly 6t^5 - 15t^4 + 10t^3
    //double alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;

    // interpolate
    //_q_ref = _q_start + alpha * (_q_home - _q_start);

    _v_start << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    _v_start = _v_start * 100;

    
    _robot->sense();
    _robot->getPositionReference(_q_start);
    jerror("Current Position '{}' \n",_q_start);
    
    // send reference
    _robot->setVelocityReference(_v_start);
    
    _robot->move();
    
    // if trajectory ended, we stop ourselves
    //if(tau >= 1.0)
    //{
    //    stop();
    //    return;
    //}

    // increment fake time
    // note: getPeriodSec() returns the nominal period
    //_fake_time += getPeriodSec();
    
    
}

XBOT2_REGISTER_PLUGIN(TestNode, testnode)
