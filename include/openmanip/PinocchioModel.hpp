#ifndef OPENMANIP_PINOCCHIOMODEL_HPP
#define OPENMANIP_PINOCCHIOMODEL_HPP

#include "logger.hpp"
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <vector>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/explog.hpp"

namespace openmanip {

    class KinematicsEngine {
        public:
            KinematicsEngine();
            ~KinematicsEngine();

            bool initialize(const std::string& urdf_path);
            // update the robot pose
            void update(const Eigen::VectorXd& q);

            // fw kin 
            Eigen::Matrix4d getFramePose(const std::string& frame_name) const;
            
            // Jcb (6 x nq)
            Eigen::MatrixXd getFrameJacobian(const std::string& frame_name) const;
            
            Eigen::VectorXd integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& v, double dt);

            Eigen::VectorXd computeTwistError(const std::string& frame_name, const Eigen::Matrix4d& target_pose);
            void printFrames();
        
        private:
            mutable Logger log;
            std::unique_ptr<pinocchio::Model> model_ = nullptr;
            std::unique_ptr<pinocchio::Data> data_ = nullptr;
    };
}

#endif // OPENMANIP_PINOCCHIOMODEL_HPP
