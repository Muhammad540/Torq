#ifndef OPENMANIP_PINOCCHIOMODEL_HPP
#define OPENMANIP_PINOCCHIOMODEL_HPP

#include "logger.hpp"
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <vector>


namespace pinocchio {
        template<typename _Scalar, int _Options> 
        struct JointCollectionDefaultTpl;

        template<typename _Scalar,int _Options,template<typename S, int O> class JointCollectionTpl>
        struct ModelTpl;

        template<typename _Scalar,int _Options,template<typename S, int O> class JointCollectionTpl>
        struct DataTpl;

        using Model = ModelTpl<double, 0, JointCollectionDefaultTpl>;
        using Data = DataTpl<double, 0, JointCollectionDefaultTpl>;
}

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
            
            void printFrames();
        
        private:
            Logger log;
            std::unique_ptr<pinocchio::Model> model_ = nullptr;
            std::unique_ptr<pinocchio::Data> data_ = nullptr;
    };
}

#endif // OPENMANIP_PINOCCHIOMODEL_HPP