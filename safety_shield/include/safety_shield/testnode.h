 	
#ifndef TESTNODE_H
#define TESTNODE_H

#include <xbot2/xbot2.h>

#include <vector>
#include "safety_shield/safety_shield.h"
#include "safety_shield/human_reach.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/verify.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "reach_lib.hpp"

using namespace XBot;
class TestNode : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;

    void starting() override;

    void run() override;


private:
    
    Eigen::VectorXd _q_start, _v_start,_q_ref,_q_home;
    //safety_shield::SafetyShield _shield;
    //std::vector<reach_lib::Point> _dummy_human_meas(21);
    int _iteration = 0;
    double t = 0.0;
    void set_control_mode();
    double _homing_time;
    double _fake_time;
    
};





#endif //TESTNODE_H

