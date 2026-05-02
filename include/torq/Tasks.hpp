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
     * @brief Abstract base class for kinematic tasks that contribute to the QP objective.
     *
     * Each task defines an error \f$e(q) \in \mathbb{R}^k\f$ and a Jacobian
     * \f$J(q) \in \mathbb{R}^{k \times n_v}\f$ such that the first-order task
     * dynamics are:
     * \f[
     *   J(q)\,\Delta q = -\alpha\,e(q)
     * \f]
     *
     * The base class converts this into a QP objective contribution:
     * \f[
     *   \min_{\Delta q}\;\tfrac{1}{2}\left\|W\bigl(J\,\Delta q + \alpha\,e\bigr)\right\|^2
     * \f]
     * which expands to \f$\tfrac{1}{2}\,\Delta q^\top H\,\Delta q + c^\top \Delta q\f$.
     *
     * @see @ref qp_formulation for how task contributions compose.
     * @see @ref tasks_page for per-task mathematical derivations.
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
             * @brief Compute this task's \f$(H, c)\f$ contribution to the QP objective.
             *
             * Implements:
             * \f[
             *   \bar{J} = W\,J, \quad \bar{e} = W(-\alpha\,e)
             * \f]
             * \f[
             *   \mu^{\text{eff}} = \mu \cdot \bar{e}^\top \bar{e}
             * \f]
             * \f[
             *   H = \bar{J}^\top \bar{J} + \mu^{\text{eff}}\,I_{n_v}, \quad
             *   c = -\bar{J}^\top \bar{e}
             * \f]
             *
             * @param config  Current robot configuration.
             * @return Pair (H, c) where H is \f$n_v \times n_v\f$ and c is \f$n_v \times 1\f$.
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
     * @brief Regulate the SE(3) pose of a robot frame to a desired target.
     *
     * The 6-D error is computed via the logarithmic map:
     * \f[
     *   e = \log_6\!\left(T_{\text{cur}}^{-1}\;T_{\text{des}}\right)
     *     = \begin{bmatrix} e_{\text{pos}} \\ e_{\text{ori}} \end{bmatrix}
     *     \in \mathbb{R}^6
     * \f]
     *
     * The 6-D cost vector is \f$[w_{\text{pos}}^{(3)},\;w_{\text{ori}}^{(3)}]\f$,
     * allowing independent weighting of position and orientation tracking.
     *
     * When no target is set the task is **inert** (contributes nothing to the QP).
     *
     * @see @ref tasks_page for the full derivation.
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
     * @brief Regulate joint angles toward a desired posture.
     *
     * The error is \f$e = q_{\text{ref}} \ominus q_{\text{cur}}\f$ (manifold difference)
     * and the Jacobian is the identity on actuated joints (floating-base coordinates excluded).
     *
     * Typically used as a low-priority regularisation task to keep the arm
     * near a sensible configuration and avoid singularities.
     *
     * When no target is set the task is **inert**.
     *
     * @see @ref tasks_page for the full derivation.
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
     * @brief Minimise joint velocities — viscous damping.
     *
     * Error is always zero and the Jacobian is the identity on actuated joints.
     * This penalises any motion: \f$H_{\text{damp}} = w_d^2\,I_{n_v}\f$.
     *
     * No target is needed; the task is always active once constructed.
     *
     * @see @ref tasks_page for the full derivation.
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
