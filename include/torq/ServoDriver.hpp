#ifndef TORQ_SERVO_DRIVER_HPP
#define TORQ_SERVO_DRIVER_HPP

/**
 * @file ServoDriver.hpp
 * @brief Real-hardware driver for STS3215 serial bus servos.
 *
 * 1 tick = (2*PI/4096) radians.
 * The calibration file defines the direction, raw_center, and raw_min / raw_max.
 * raw_zero: all joints use the center anchor except the gripper , it uses raw_min/max with mj_low as the anchor (asymmetric jaw):
 *   Center (for non-gripper joints): raw_zero = round(raw_center - direction * mj_center_ticks)
 *   dir==+1 -> raw_zero = raw_min - round(mj_low * STS_TICKS_PER_RAD)
 *   dir==-1 -> raw_zero = raw_max + round(mj_low * STS_TICKS_PER_RAD)
 *
 *   rawToRadian: direction * (raw - raw_zero) * STS_RADIAN_PER_TICK
 *   radianToRaw: raw_zero + (radians * STS_TICKS_PER_RAD) / direction
 *
 * Hardware safety is enforced by EEPROM min/max limits and homing offset
 * flashed during calibration, not by software clamping.
 */

#include "torq/HardwareInterface.hpp"
#include "torq/logger.hpp"
#include "torq/scservo/SMS_STS_Port.hpp"

#include <string>
#include <vector>
#include <Eigen/Core>

namespace torq {

    constexpr int    STS_POSITION_MIN    = 0;
    constexpr int    STS_POSITION_MAX    = 4095;
    constexpr int    STS_ENCODER_TICKS   = 4096;
    constexpr double STS_TICKS_PER_RAD   = STS_ENCODER_TICKS / (2.0 * 3.14159265358979323846);
    constexpr double STS_RADIAN_PER_TICK = (2.0 * 3.14159265358979323846) / STS_ENCODER_TICKS;

    /// Hardcoded from MuJoCo XML  (for so101)
    struct JointSpec {
        std::string name;
        double mj_low;
        double mj_high;
    };

    /// Per-joint calibration data.
    struct JointCalibration {
        int servo_id = 0;
        std::string joint_name;
        int direction   = 1;      ///< +1 or -1
        int raw_center  = 2048;   ///< After EEPROM homing
        int raw_min     = 0;      ///< Sweep low tick
        int raw_max     = 4095;   ///< Sweep high tick
        int raw_zero    = 2048;   ///< Computed at connect()
    };

    class ServoDriver : public HardwareInterface {
    public:
        ServoDriver();
        ~ServoDriver() override;

        ServoDriver(const ServoDriver&) = delete;
        ServoDriver& operator=(const ServoDriver&) = delete;

        bool connect(const std::string& config_path) override;
        void disconnect() override;

        Eigen::VectorXd getJointPositions() const override;
        Eigen::VectorXd getJointVelocities() const override;
        Eigen::VectorXd getCtrl() const override;

        void setJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q) override;
        void setJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd) override;

        void step() override;
        double getTimestep() const override;
        int numJoints() const override;
        int numActuators() const override;

        bool isConnected() const { return connected_; }

        bool pingServo(int id);
        bool enableTorque(int id, bool enable);
        void enableAllTorque(bool enable);
        bool readPositionsSyncRead();

    private:
        bool openSerial();
        void closeSerial();
        bool readPositions();
        bool writePositions(const Eigen::Ref<const Eigen::VectorXd>& q);

        double rawToRadian(int raw, int index) const;
        int radianToRaw(double rad, int index) const;

        std::string port_;
        int baud_rate_         = 1000000;
        double control_period_ = 0.002;
        int default_speed_     = 2000;

        std::vector<JointCalibration> calibration_;
        std::vector<JointSpec> joint_specs_;
        std::vector<int> servo_ids_;

        bool connected_ = false;
        int serial_fd_  = -1;
        scservo::SMS_STS_Port sts_;

        Eigen::VectorXd q_current_;
        Eigen::VectorXd qd_current_;
        Eigen::VectorXd ctrl_last_;

        Logger log_;
    };

} // namespace torq

#endif // TORQ_SERVO_DRIVER_HPP
