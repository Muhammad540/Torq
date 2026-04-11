#include "torq/ServoDriver.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cerrno>

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#endif

namespace torq {

static const std::vector<JointSpec> SO101_JOINT_SPECS = {
    {"shoulder_pan",  -1.9198621771937616,  1.9198621771937634},
    {"shoulder_lift", -1.7453292519943224,  1.7453292519943366},
    {"elbow_flex",    -1.69,                1.69},
    {"wrist_flex",    -1.6580628494556928,  1.6580627293335335},
    {"wrist_roll",    -2.7438472969992493,  2.841206309382605},
    {"gripper",       -0.17453297762778586, 1.7453291995659765},
};

static const JointSpec* findSpec(const std::string& name) {
    for (const auto& s : SO101_JOINT_SPECS)
        if (s.name == name) return &s;
    return nullptr;
}

ServoDriver::ServoDriver() {
    log_.info() << "[ServoDriver] Created (use connect() to open)";
}

ServoDriver::~ServoDriver() {
    disconnect();
}

// ─── Serial port ──────────────────────────────────────────

bool ServoDriver::openSerial() {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        log_.error() << "[ServoDriver] Failed to open " << port_ << ": " << strerror(errno);
        return false;
    }

    fcntl(serial_fd_, F_SETFL, 0);

    struct termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
        log_.error() << "[ServoDriver] tcgetattr failed";
        close(serial_fd_); serial_fd_ = -1;
        return false;
    }

    speed_t speed;
    switch (baud_rate_) {
        case 57600:   speed = B57600;   break;
        case 115200:  speed = B115200;  break;
        case 500000:  speed = B500000;  break;
        case 1000000: speed = B1000000; break;
        default:
            log_.error() << "[ServoDriver] Unsupported baud: " << baud_rate_;
            close(serial_fd_); serial_fd_ = -1;
            return false;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        log_.error() << "[ServoDriver] tcsetattr failed";
        close(serial_fd_); serial_fd_ = -1;
        return false;
    }

    tcflush(serial_fd_, TCIOFLUSH);
    log_.info() << "[ServoDriver] Opened " << port_ << " @ " << baud_rate_ << " baud";
    return true;
}

void ServoDriver::closeSerial() {
    if (serial_fd_ >= 0) { close(serial_fd_); serial_fd_ = -1; }
}

// ─── Connect / Disconnect ─────────────────────────────────

bool ServoDriver::connect(const std::string& config_path) {
    if (connected_) disconnect();

    port_.clear();
    baud_rate_      = 1000000;
    control_period_ = 0.002;
    default_speed_  = 2000;
    calibration_.clear();
    joint_specs_.clear();
    servo_ids_.clear();

    std::ifstream f(config_path);
    if (!f.is_open()) {
        log_.error() << "[ServoDriver] Cannot open config: " << config_path;
        return false;
    }

    std::string line;
    while (std::getline(f, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
            line.pop_back();
        if (line.empty() || line[0] == '#') continue;

        size_t eq = line.find('=');
        if (eq != std::string::npos) {
            std::string key = line.substr(0, eq);
            std::string val = line.substr(eq + 1);
            if      (key == "port")           port_ = val;
            else if (key == "baud_rate")      baud_rate_ = std::stoi(val);
            else if (key == "control_period") control_period_ = std::stod(val);
            else if (key == "default_speed")  default_speed_ = std::stoi(val);
            continue;
        }

        std::istringstream ss(line);
        int joint_index;
        JointCalibration jc;
        if (!(ss >> joint_index >> jc.servo_id >> jc.joint_name
                 >> jc.direction >> jc.raw_center))
            continue;

        const JointSpec* spec = findSpec(jc.joint_name);
        if (!spec) {
            log_.warning() << "[ServoDriver] Unknown joint '" << jc.joint_name
                           << "', skipping";
            continue;
        }

        auto apply_center_anchor = [&]() {
            jc.raw_min = 0;
            jc.raw_max = 4095;
            double mj_center_rad = (spec->mj_low + spec->mj_high) / 2.0;
            double mj_center_ticks = mj_center_rad * STS_TICKS_PER_RAD;
            jc.raw_zero = static_cast<int>(std::round(jc.raw_center - jc.direction * mj_center_ticks));
        };

        if (jc.joint_name == "gripper") {
            int sweep_lo = 0;
            int sweep_hi = 0;
            bool has_sweep = false;
            if (ss >> sweep_lo >> sweep_hi) {
                if (sweep_hi > sweep_lo && !(sweep_lo <= 1 && sweep_hi >= 4090))
                    has_sweep = true;
            }
            if (has_sweep) {
                jc.raw_min = sweep_lo;
                jc.raw_max = sweep_hi;
                double mj_low_ticks = spec->mj_low * STS_TICKS_PER_RAD;
                if (jc.direction == 1)
                    jc.raw_zero = jc.raw_min - static_cast<int>(std::round(mj_low_ticks));
                else
                    jc.raw_zero = jc.raw_max + static_cast<int>(std::round(mj_low_ticks));
            } else {
                apply_center_anchor();
            }
        } else {
            apply_center_anchor();
        }

        while (static_cast<int>(calibration_.size()) < joint_index) {
            calibration_.push_back(JointCalibration{});
            joint_specs_.push_back(JointSpec{});
        }

        calibration_.push_back(jc);
        joint_specs_.push_back(*spec);
        servo_ids_.push_back(jc.servo_id);
    }

    if (port_.empty()) {
        log_.error() << "[ServoDriver] No 'port' specified in " << config_path;
        return false;
    }

    if (servo_ids_.empty()) {
        log_.warning() << "[ServoDriver] No calibration data";
        return false;
    }

    int n = static_cast<int>(servo_ids_.size());
    q_current_  = Eigen::VectorXd::Zero(n);
    qd_current_ = Eigen::VectorXd::Zero(n);
    ctrl_last_  = Eigen::VectorXd::Zero(n);

    if (!openSerial()) return false;

    sts_.setPort(serial_fd_);
    sts_.IOTimeOut = 500;
    sts_.End = 0;
    connected_ = true;

    int found = 0;
    for (int i = 0; i < n; ++i) {
        if (pingServo(servo_ids_[i]))
            ++found;
        else
            log_.warning() << "[ServoDriver] Servo ID " << servo_ids_[i] << " did not respond";
    }

    if (found == 0) {
        log_.error() << "[ServoDriver] No servos responded";
        disconnect();
        return false;
    }

    readPositions();
    ctrl_last_ = q_current_;

    log_.info() << "[ServoDriver] Connected, " << found << "/" << n
                << " servos, " << calibration_.size() << " calibrated joints";
    return true;
}

void ServoDriver::disconnect() {
    if (!connected_) return;
    sts_.setPort(-1);
    closeSerial();
    connected_ = false;
    log_.info() << "[ServoDriver] Disconnected";
}


double ServoDriver::rawToRadian(int raw, int index) const {
    if (index >= 0 && index < static_cast<int>(calibration_.size())) {
        const auto& cal = calibration_[index];
        return cal.direction * (raw - cal.raw_zero) * STS_RADIAN_PER_TICK;
    }
    return (raw - 2048) * STS_RADIAN_PER_TICK;
}

int ServoDriver::radianToRaw(double rad, int index) const {
    if (index >= 0 && index < static_cast<int>(calibration_.size())) {
        const auto& cal = calibration_[index];
        int raw = cal.raw_zero + static_cast<int>(
            std::round(rad * STS_TICKS_PER_RAD / cal.direction));
        return std::clamp(raw, STS_POSITION_MIN, STS_POSITION_MAX);
    }
    int raw = 2048 + static_cast<int>(std::round(rad * STS_TICKS_PER_RAD));
    return std::clamp(raw, STS_POSITION_MIN, STS_POSITION_MAX);
}

bool ServoDriver::readPositions() {
    if (!connected_ || serial_fd_ < 0) return false;

    int n = static_cast<int>(servo_ids_.size());
    for (int i = 0; i < n; ++i) {
        int raw = sts_.ReadPos(servo_ids_[i]);
        if (raw < 0) {
            log_.warning() << "[ServoDriver] Read failed for ID " << servo_ids_[i];
            continue;
        }
        double prev = q_current_(i);
        q_current_(i) = rawToRadian(raw, i);
        if (control_period_ > 0)
            qd_current_(i) = (q_current_(i) - prev) / control_period_;
    }
    return true;
}

bool ServoDriver::readPositionsSyncRead() {
    if (!connected_ || serial_fd_ < 0) return false;

    int n = static_cast<int>(servo_ids_.size());
    if (n == 0 || n > 6) return false;

    u8 ids[6];
    for (int i = 0; i < n; ++i)
        ids[i] = static_cast<u8>(servo_ids_[i]);

    sts_.rFlushSCS();
    sts_.syncReadPacketTx(ids, static_cast<u8>(n), 56, 2);
    sts_.wFlushSCS();

    for (int i = 0; i < n; ++i) {
        u8 buf[2];
        if (sts_.syncReadPacketRx(ids[i], buf) != 2) {
            log_.warning() << "[ServoDriver] SyncRead: no reply from ID " << servo_ids_[i];
            continue;
        }
        int raw = sts_.syncReadRxPacketToWrod(15);
        if (raw < 0) continue;
        double prev = q_current_(i);
        q_current_(i) = rawToRadian(raw, i);
        if (control_period_ > 0)
            qd_current_(i) = (q_current_(i) - prev) / control_period_;
    }
    return true;
}

bool ServoDriver::writePositions(const Eigen::Ref<const Eigen::VectorXd>& q) {
    if (!connected_ || serial_fd_ < 0) return false;

    int n = static_cast<int>(servo_ids_.size());
    if (q.size() != n || n > 6) return false;

    u8  ids[6];
    s16 positions[6];
    u16 speeds[6];
    u8  accs[6];

    for (int i = 0; i < n; ++i) {
        ids[i]       = static_cast<u8>(servo_ids_[i]);
        positions[i] = static_cast<s16>(radianToRaw(q(i), i));
        speeds[i]    = static_cast<u16>(default_speed_);
        accs[i]      = 0;
    }

    sts_.SyncWritePosEx(ids, static_cast<u8>(n), positions, speeds, accs);
    return true;
}

// ─── Public API ───────────────────────────────────────────

Eigen::VectorXd ServoDriver::getJointPositions() const {
    return connected_ ? q_current_ : Eigen::VectorXd::Zero(0);
}

Eigen::VectorXd ServoDriver::getJointVelocities() const {
    return connected_ ? qd_current_ : Eigen::VectorXd::Zero(0);
}

Eigen::VectorXd ServoDriver::getCtrl() const {
    return connected_ ? ctrl_last_ : Eigen::VectorXd::Zero(0);
}

void ServoDriver::setJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q) {
    if (!connected_) return;
    if (writePositions(q))
        ctrl_last_ = q;
}

void ServoDriver::setJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd) {
    if (!connected_) return;
    int n = static_cast<int>(qd.size());
    if (n != static_cast<int>(ctrl_last_.size())) return;

    Eigen::VectorXd q_new = ctrl_last_ + qd * control_period_;

    for (int i = 0; i < n; ++i) {
        if (i < static_cast<int>(joint_specs_.size())) {
            // soft clamping
            q_new(i) = std::clamp(q_new(i),
                                  joint_specs_[i].mj_low,
                                  joint_specs_[i].mj_high);
        }
    }

    setJointPositions(q_new);
}

void ServoDriver::step() {
    if (connected_) readPositions();
}

double ServoDriver::getTimestep() const { return control_period_; }

int ServoDriver::numJoints() const {
    return connected_ ? static_cast<int>(servo_ids_.size()) : 0;
}

int ServoDriver::numActuators() const { return numJoints(); }

bool ServoDriver::pingServo(int id) {
    int ret = sts_.Ping(static_cast<u8>(id));
    return (ret >= 0 && ret == id);
}

bool ServoDriver::enableTorque(int id, bool enable) {
    return sts_.EnableTorque(static_cast<u8>(id), enable ? 1 : 0) != 0;
}

void ServoDriver::enableAllTorque(bool enable) {
    for (int id : servo_ids_)
        sts_.EnableTorque(static_cast<u8>(id), enable ? 1 : 0);
}

} // namespace torq
