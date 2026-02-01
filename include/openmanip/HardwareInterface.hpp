#ifndef OPENMANIP_HARDWARE_INTERFACE_HPP
#define OPENMANIP_HARDWARE_INTERFACE_HPP

#include <Eigen/Core>
#include <string>

namespace openmanip {
    // Abstract Interface for any robot drive (Sim or real)
    class HardwareInterface{
        public:
            virtual ~HardwareInterface() = default;
            
            virtual bool connect(const std::string& config_path) = 0;
            virtual void disconnect() = 0;

            virtual Eigen::VectorXd getJointPositions() const = 0;
            virtual Eigen::VectorXd getJointVelocities() const = 0;
            
            virtual void overrideJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q) = 0;
            virtual void overrideJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd) = 0;
            
            virtual void step() = 0;
            virtual double getTimestep() const = 0;

            virtual int numJoints() const = 0;
    };

} // namespace openmanip

#endif // OPENMANIP_HARDWARE_INTERFACE_HPP