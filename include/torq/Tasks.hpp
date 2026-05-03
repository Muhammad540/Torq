#ifndef TORQ_TASKS_HPP
#define TORQ_TASKS_HPP

#include <Eigen/Dense>
#include <utility>
#include <optional>
#include <string>
#include <vector>
#include <pinocchio/spatial/se3.hpp>


namespace torq{
    class Configuration;

    /**
     * @brief Abstract base for QP objective tasks.
     *
     * A task defines an error \f$e(q) \in \mathbb{R}^k\f$ and Jacobian
     * \f$J(q) \in \mathbb{R}^{k \times n_v}\f$ with first-order dynamics
     * \f$J\,\Delta q = -\alpha\,e\f$. The base class assembles the QP block
     * \f$\tfrac{1}{2}\,\Delta q^\top H\,\Delta q + c^\top \Delta q\f$ via
     * computeQPObjective().
     *
     * @see @ref tasks_page, @ref qp_formulation
     */
    class Task{
        public:
            /**
             * @brief Construct with a vector cost (one weight per error dimension).
             * @param cost  Weight vector \f$w \in \mathbb{R}^k\f$ determining this task's
             *              importance relative to other tasks.
             * @param gain  Task gain \f$\alpha \in (0, 1]\f$.  1.0 = dead-beat (full correction
             *              each tick); lower values low-pass filter the response.
             * @param lm_damping  Levenberg–Marquardt damping \f$\mu \ge 0\f$.  Increase when
             *                    the target is unreachable to avoid velocity spikes.
             */
            Task(const Eigen::VectorXd& cost, double gain = 1.0, double lm_damping= 0.0);

            /**
             * @brief Construct with a scalar cost (broadcast to match error dimension).
             * @param cost  Scalar weight applied uniformly to all error components.
             * @param gain  Task gain \f$\alpha\f$.
             * @param lm_damping  Levenberg–Marquardt damping \f$\mu\f$.
             */
            Task (double cost, double gain = 1.0, double lm_damping = 0.0);

            virtual ~Task() = default;
            
            /**
             * @brief Compute the task error \f$e(q) \in \mathbb{R}^k\f$.
             * @param config  Current robot configuration (FK already computed).
             * @return Error vector whose dimension depends on the concrete task type.
             */
            virtual Eigen::VectorXd computeError(const Configuration& config) const = 0;

            /**
             * @brief Compute the task Jacobian \f$J(q) \in \mathbb{R}^{k \times n_v}\f$.
             * @param config  Current robot configuration.
             * @return Jacobian matrix mapping joint displacements to task-space changes.
             */
            virtual Eigen::MatrixXd computeJacobian(const Configuration& config) const = 0;

            /**
             * @brief Assemble this task's \f$(H, c)\f$ contribution to the QP.
             *
             * Implements the weighted least-squares form with adaptive LM
             * damping. See @ref qp_formulation for the full derivation.
             *
             * @return Pair (H, c) of size \f$(n_v \times n_v)\f$ and \f$n_v\f$.
             */
            std::pair<Eigen::MatrixXd, Eigen::VectorXd> computeQPObjective(const Configuration& config) const;

            /** @brief Set a vector cost (one weight per error dimension). */
            void setCost(const Eigen::VectorXd& cost) { cost_ = cost; scalar_cost_ = false; }
            /** @brief Set a scalar cost (broadcast to all error dimensions). */
            void setCost(double cost) { cost_ = Eigen::VectorXd::Constant(1, cost); scalar_cost_ = true; }
            /** @brief Set the task gain \f$\alpha \in (0, 1]\f$. */
            void setGain(double gain) { gain_ = gain; }
            /** @brief Set the Levenberg–Marquardt damping \f$\mu \ge 0\f$. */
            void setLMDamping(double lm_damping) { lm_damping_ = lm_damping; }

            /** @brief Current gain value. */
            double gain() const { return gain_; }
            /** @brief Current LM damping value. */
            double lmDamping() const { return lm_damping_; }
            /** @brief Current cost vector. */
            const Eigen::VectorXd& cost() const { return cost_; }
            /** @brief True if the cost was set as a scalar (single element). */
            bool isScalarCost() const { return scalar_cost_; }
            /** @brief First element of the cost vector (the scalar cost if isScalarCost()). */
            double scalarCost() const { return cost_(0); }

        protected:
            Eigen::VectorXd cost_;
            double gain_;
            double lm_damping_;
            bool scalar_cost_;
    };

    /**
     * @brief Track the SE(3) pose of a frame.
     *
     * 6-D error from the SE(3) logarithm
     * \f$e = \log_6(T_\text{cur}^{-1} T_\text{des})\f$, with separate
     * position / orientation weights. Inert (zero contribution) until a
     * target is set.
     *
     * @see @ref tasks_page
     */
    class FrameTask : public Task {
        public:
            /**
            * @brief Construct a FrameTask for the named frame.
            * @param frame            Name of the frame in the Pinocchio model.
            * @param position_cost    Scalar weight for translational error [cost/m].
            * @param orientation_cost Scalar weight for rotational error [cost/rad].
            * @param lm_damping       Levenberg–Marquardt damping.
            * @param gain             Task gain \f$\alpha\f$.
            */
            FrameTask(const std::string& frame,
                    double position_cost,
                    double orientation_cost,
                    double lm_damping = 0.0,
                    double gain = 1.0);

            /**
            * @brief Set the desired SE(3) target pose.
            * @param target  Desired transform \f$T_{\text{des}} \in SE(3)\f$.
            */
            void setTarget(const pinocchio::SE3& target);

            /**
            * @brief Set the target from the current pose of the frame in the given configuration.
            * @param config  Configuration whose FK has been computed.
            */
            void setTargetFromConfiguration(const Configuration& config);

            /** @brief Set scalar position cost (broadcast to all 3 translational axes). */
            void setPositionCost(double cost);
            /** @brief Set per-axis position cost. */
            void setPositionCost(const Eigen::Vector3d& cost);
            /** @brief Set scalar orientation cost (broadcast to all 3 rotational axes). */
            void setOrientationCost(double cost);
            /** @brief Set per-axis orientation cost. */
            void setOrientationCost(const Eigen::Vector3d& cost);

            /** @brief True if a target has been set. */
            bool hasTarget() const { return target_.has_value(); }
            /** @brief Desired SE(3) target when set (read-only). */
            const std::optional<pinocchio::SE3>& targetTransform() const { return target_; }
            /** @brief Name of the controlled frame. */
            const std::string& frame() const { return frame_; }
            /** @brief Current scalar position cost (first element of the cost vector). */
            double positionCost() const { return cost_(0); }
            /** @brief Current scalar orientation cost (fourth element of the cost vector). */
            double orientationCost() const { return cost_(3); }

            Eigen::VectorXd computeError(const Configuration& config) const override;
            Eigen::MatrixXd computeJacobian(const Configuration& config) const override;

        private:
            std::string frame_;
            std::optional<pinocchio::SE3> target_;
    };

    /**
     * @brief Bias joints toward a reference posture.
     *
     * Error is the manifold difference \f$q_\text{ref} \ominus q_\text{cur}\f$
     * (floating-base coordinates excluded); Jacobian is the identity on
     * actuated joints. Used as a low-priority regulariser. Inert until a
     * target is set.
     *
     * @see @ref tasks_page
     */
    class PostureTask : public Task {
        public:
            /**
             * @brief Construct a PostureTask.
             * @param cost       Scalar cost [cost/rad] broadcast to all actuated joints.
             * @param lm_damping Levenberg–Marquardt damping.
             * @param gain       Task gain \f$\alpha\f$.
             */
            PostureTask(double cost, double lm_damping = 0.0, double gain = 1.0);

            /**
             * @brief Set the reference posture.
             * @param target_q  Full configuration vector (nq), including floating-base coords if present.
             */
            void setTarget(const Eigen::VectorXd& target_q);

            /**
             * @brief Set the reference posture from the current configuration.
             * @param config  Configuration whose q will be used as the reference.
             */
            void setTargetFromConfiguration(const Configuration& config);

            /** @brief True if a reference posture has been set. */
            bool hasTarget() const { return target_q_.has_value(); }

            Eigen::VectorXd computeError(const Configuration& config) const override;
            Eigen::MatrixXd computeJacobian(const Configuration& config) const override;
    
        private:
            std::optional<Eigen::VectorXd> target_q_;
    };

    /**
     * @brief Penalise joint velocity (viscous damping).
     *
     * Zero error, identity Jacobian on actuated joints — adds
     * \f$w^2\,I\f$ to the QP Hessian. Always active.
     *
     * @see @ref tasks_page
     */
    class DampingTask : public Task {
        public:
        /**
         * @brief Construct a DampingTask.
         * @param cost  Scalar cost [cost·s/rad] penalising joint velocity magnitude.
         */
        DampingTask(double cost);
        Eigen::VectorXd computeError(const Configuration& config) const override;
        Eigen::MatrixXd computeJacobian(const Configuration& config) const override;
    };
}

#endif // TORQ_TASK_HPP
