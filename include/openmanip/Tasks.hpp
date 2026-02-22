#ifndef OPENMANIP_TASKS_HPP
#define OPENMANIP_TASKS_HPP

#include <Eigen/Dense>
#include <utility>
#include <optional>
#include <string>
#include <pinocchio/spatial/se3.hpp>


namespace openmanip{
    class Configuration;

    /*
        Abstract base class for kinematic class.

        Each task defines an error e(q) and jacobian j(q) such that the first order task dynamics are: J(q) * dq = -alpha * e(q)

        The base class converts this into a QP objective contribution:
            min 0.5 * || W * (J*dq + alpha*e) ||^2
        which expands to:  0.5 * dq^T * H * dq + c^T * dq

        Cost: weight vector that determines the task's importance relative to other tasks
        Gain: Task's gain for low pass filtering (1.0 for dead beat control, lower for more filtering)
        lm_damping: To help with infeasible motions.
        nq: Dimension of the position/configuration vector.
        nv: Dimension of the velocity vector (tangent space).
    */
    class Task{
        public:
            /* construct the vector cost (e.g. 6D for FrameTask)*/
            Task(const Eigen::VectorXd& cost, double gain = 1.0, double lm_damping= 0.0);

            /* Construct with scalar cost (broadcast to match error dimension) */
            Task (double cost, double gain = 1.0, double lm_damping = 0.0);

            virtual ~Task() = default;
            
            /* Compute Task error e(q). Dimension k depends on the task type */
            virtual Eigen::VectorXd computeError(const Configuration& config) const = 0;

            /* Compute Task Jacobian J(q), shape (k x nv) */
            virtual Eigen::MatrixXd computeJacobian(const Configuration& config) const = 0;

            /* 
            Compute this task's (H, c) contribution to the QP objective.
                H is (nv x nv), c is (nv x 1).

                Implements:
                    WJ = W * J
                    We = W * (-alpha * e)
                    mu = lm_damping * (We^T We)           // LM regularization
                    H  = WJ^T * WJ  +  mu * I_{nv}
                    c  = -WJ^T * We
            */
            std::pair<Eigen::MatrixXd, Eigen::VectorXd> computeQPObjective(const Configuration& config) const;

            void setGain(double gain) {gain_ = gain; }
            void setLMDamping(double lm_damping) {lm_damping_ = lm_damping; }

            double gain() const { return gain_; }
            double lmDamping() const { return lm_damping_; }
            const Eigen::VectorXd& cost() const { return cost_; }

        protected:
            Eigen::VectorXd cost_;
            double gain_;
            double lm_damping_;
            bool scalar_cost_;
    };

    /*
        Task to set the SE3 pose of the robot frame in the world frame. 

        The 6D cost vector is [position_cost(3), orientation_cost(3)]
        When target is not set, the task is inert (contributes nothing to the QP)
    */
    class FrameTask : public Task {
    public:
        FrameTask(const std::string& frame,
                double position_cost,
                double orientation_cost,
                double lm_damping = 0.0,
                double gain = 1.0);

        void setTarget(const pinocchio::SE3& target);
        void setTargetFromConfiguration(const Configuration& config);

        void setPositionCost(double cost);
        void setPositionCost(const Eigen::Vector3d& cost);
        void setOrientationCost(double cost);
        void setOrientationCost(const Eigen::Vector3d& cost);

        bool hasTarget() const { return target_.has_value(); }
        const std::string& frame() const { return frame_; }

        Eigen::VectorXd computeError(const Configuration& config) const override;
        Eigen::MatrixXd computeJacobian(const Configuration& config) const override;

    private:
        std::string frame_;
        std::optional<pinocchio::SE3> target_;
    };

    /*
        Regulate joint angles towards a desired posture

        Floating base coordinates are excluded. The scalar cost is broadcast uniformly across all actuated joints.
        Typically used as a low priority regularization task to avoid singularities. 

        When target is not set, the task is inert.
    */
    class PostureTask : public Task {
        public:
            PostureTask(double cost, double lm_damping = 0.0, double gain = 1.0);

            /* target_q must be full nq (including floating base coords if present)*/
            void setTarget(const Eigen::VectorXd& target_q);
            void setTargetFromConfiguration(const Configuration& config);

            bool hasTarget() const { return target_q_.has_value(); }

            Eigen::VectorXd computeError(const Configuration& config) const override;
            Eigen::MatrixXd computeJacobian(const Configuration& config) const override;
    
        private:
            std::optional<Eigen::VectorXd> target_q_;
    };

    /*
        Minimize the joint velocities (bring robot to rest)

        Error is always zero, Jacobian is identity on actuated joints.
        This penalizes any motion, acting as viscous damping.
        No target needed, always active once constructed
    */
    class DampingTask : public Task {
        public:
        DampingTask(double cost);
        Eigen::VectorXd computeError(const Configuration& config) const override;
        Eigen::MatrixXd computeJacobian(const Configuration& config) const override;
    };
}

#endif // OPENMANIP_TASK_HPP
