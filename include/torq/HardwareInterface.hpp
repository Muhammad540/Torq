#ifndef TORQ_HARDWARE_INTERFACE_HPP
#define TORQ_HARDWARE_INTERFACE_HPP

#include <Eigen/Core>
#include <string>

namespace torq {

    /**
     * @brief Abstract hardware abstraction layer for any robot driver (simulation or real).
     *
     * All control and kinematics code targets this interface.  Swapping the
     * concrete implementation (e.g. MujocoDriver → a real hardware driver)
     * enables sim-to-real transfer without touching control logic.
     *
     * Two "spaces" are exposed:
     * - **State space** (`getJointPositions` / `getJointVelocities`): \f$q \in \mathbb{R}^{n_q}\f$,
     *   \f$\dot{q} \in \mathbb{R}^{n_v}\f$.
     * - **Actuator command space** (`setJointPositions` / `setJointVelocities`, `getCtrl`):
     *   may differ from state space (e.g. underactuated systems, gear ratios).
     *
     * @see MujocoDriver
     */
    class HardwareInterface{
        public:
            virtual ~HardwareInterface() = default;
            
            /**
             * @brief Connect to the hardware (or load a simulation scene).
             * @param config_path  Path to a configuration / model file.
             * @return True on success.
             */
            virtual bool connect(const std::string& config_path) = 0;

            /** @brief Disconnect from the hardware / release resources. */
            virtual void disconnect() = 0;

            /** @brief Current joint positions \f$q \in \mathbb{R}^{n_q}\f$. */
            virtual Eigen::VectorXd getJointPositions() const = 0;

            /** @brief Current joint velocities \f$\dot{q} \in \mathbb{R}^{n_v}\f$. */
            virtual Eigen::VectorXd getJointVelocities() const = 0;

            /** @brief Current actuator commands (ctrl vector, size \f$n_u\f$). */
            virtual Eigen::VectorXd getCtrl() const = 0;

            /**
             * @brief Send position commands through the actuator interface.
             * @param q  Target positions (size \f$n_u\f$).
             */
            virtual void setJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q) = 0;

            /**
             * @brief Send velocity commands through the actuator interface.
             * @param qd  Target velocities (size \f$n_u\f$).
             */
            virtual void setJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd) = 0;

            /** @brief Advance the simulation / hardware by one timestep. Called at the control loop frequency. */
            virtual void step() = 0;

            /** @brief Integration timestep [s]. Physics rate is 1/getTimestep() Hz. */
            virtual double getTimestep() const = 0;

            /** @brief Number of joints (state-space dimension \f$n_q\f$). */
            virtual int numJoints() const = 0;

            /** @brief Number of actuators (command-space dimension \f$n_u\f$). */
            virtual int numActuators() const = 0;
    };

} // namespace torq

#endif // TORQ_HARDWARE_INTERFACE_HPP
