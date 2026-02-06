#ifndef OPENMANIP_INVERSE_KINEMATICS_HPP
#define OPENMANIP_INVERSE_KINEMATICS_HPP

#include <Eigen/Dense>
#include <string>
#include "openmanip/PinocchioModel.hpp"
#include "openmanip/logger.hpp"

namespace openmanip{

 struct IKConfig{
    // DLS damping factor 
    float damping = 1e-4;
    // Maximum joint velocity limit
    double velocity_limit = 10.0;
    // kp (1/s)
    float kp = 1.0; 
  };
  
class InverseKinematics{
public:
  InverseKinematics(KinematicsEngine* kinematics);
  ~InverseKinematics();

  Eigen::VectorXd solve(const Eigen::VectorXd& q_curr, const Eigen::Matrix4d& target_pos, const std::string& frame);
  
private:
  KinematicsEngine* kinematics_;
  IKConfig config_;
  Logger logger;
};

}
#endif // OPENMANIP_INVERSE_KINEMATICS_HPP
