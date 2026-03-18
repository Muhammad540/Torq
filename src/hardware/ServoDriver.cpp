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
#define TORQ_SERIAL_POSIX 1
#else
#define TORQ_SERIAL_POSIX 0
#endif

namespace torq {

// ─── ST3215 / STS specs (radian conversion and config defaults) ──────────────────────────────
namespace st3215 {
    constexpr int POSITION_MIN = 0;
    constexpr int POSITION_MAX = 4095;
    constexpr int DIGITAL_RANGE = 4096;     // 12-bit
    constexpr double DIGITAL_MIDDLE = 2047.5;
    constexpr int DEFAULT_SPEED = 2000;
    constexpr int DEFAULT_TIME = 0;
    constexpr int BAUD_RATE = 1000000;
} // namespace st3215

// ─── Helpers ──────────────────────────────────────────────
static double defaultPositionScale() {
    return (2.0 * M_PI) / st3215::DIGITAL_RANGE;
}

// ─── Constructor / Destructor ─────────────────────────────
ServoDriver::ServoDriver() {
    log_.info() << "[ServoDriver] Created (use connect() to open)";
}

ServoDriver::~ServoDriver() {
    disconnect();
}

// ─── Config ───────────────────────────────────────────────
// Parsing: if path_or_port contains .json/.yaml/.conf, open as key=value file
// (see ServoDriver.hpp "Config structure and how it is passed and parsed").
// Otherwise path_or_port is the serial port; ST3215 defaults apply.
bool ServoDriver::loadConfig(const std::string& path_or_port) {
    config_ = SerialServoConfig();
    config_.port = path_or_port;
    config_.baud_rate = st3215::BAUD_RATE;
    config_.protocol = "st3215";
    config_.control_period = 0.002;
    config_.position_min = st3215::POSITION_MIN;
    config_.position_max = st3215::POSITION_MAX;
    config_.position_scale = defaultPositionScale();
    config_.position_zero = st3215::DIGITAL_MIDDLE;
    config_.default_speed = st3215::DEFAULT_SPEED;
    config_.default_time = st3215::DEFAULT_TIME;

    // Parse simple key=value config if file extension detected
    if (path_or_port.find(".json") != std::string::npos ||
        path_or_port.find(".yaml") != std::string::npos ||
        path_or_port.find(".conf") != std::string::npos) {
        std::ifstream f(path_or_port);
        if (f.is_open()) {
            std::string line;
            while (std::getline(f, line)) {
                // Strip trailing whitespace/CR
                while (!line.empty() &&
                       (line.back() == '\r' || line.back() == ' '))
                    line.pop_back();

                size_t eq = line.find('=');
                if (eq == std::string::npos) continue;

                std::string key = line.substr(0, eq);
                std::string val = line.substr(eq + 1);

                if (key == "port")
                    config_.port = val;
                else if (key == "baud_rate")
                    config_.baud_rate = std::stoi(val);
                else if (key == "protocol")
                    config_.protocol = val;
                else if (key == "control_period")
                    config_.control_period = std::stod(val);
                else if (key == "default_speed")
                    config_.default_speed = std::stoi(val);
                else if (key == "servo_ids") {
                    config_.servo_ids.clear();
                    std::istringstream ss(val);
                    int id;
                    while (ss >> id)
                        config_.servo_ids.push_back(id);
                }
            }
        }
    }

    if (config_.servo_ids.empty()) {
        for (int i = 1; i <= 6; ++i)
            config_.servo_ids.push_back(i);
    }

    return true;
}

// ─── Serial Port ──────────────────────────────────────────
#if TORQ_SERIAL_POSIX

bool ServoDriver::openSerial() {
    // Open in blocking mode initially, we'll use select() for
    // timeouts
    serial_fd_ = open(
        config_.port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY
    );
    if (serial_fd_ < 0) {
        log_.error() << "[ServoDriver] Failed to open "
                     << config_.port << ": "
                     << strerror(errno);
        return false;
    }

    // Clear O_NDELAY so read() blocks
    fcntl(serial_fd_, F_SETFL, 0);

    struct termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
        log_.error() << "[ServoDriver] tcgetattr failed";
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Baud rate
    speed_t speed;
    switch (config_.baud_rate) {
        case 57600:   speed = B57600;   break;
        case 115200:  speed = B115200;  break;
        case 500000:  speed = B500000;  break;
        case 1000000: speed = B1000000; break;
        default:
            log_.error() << "[ServoDriver] Unsupported baud: "
                         << config_.baud_rate;
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1, no flow control
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                      INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);

    // Inter-character timeout: 1 * 100ms = 100ms max
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        log_.error() << "[ServoDriver] tcsetattr failed";
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Flush any stale data
    tcflush(serial_fd_, TCIOFLUSH);

    log_.info() << "[ServoDriver] Opened " << config_.port
                << " @ " << config_.baud_rate << " baud";
    return true;
}

void ServoDriver::closeSerial() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

#else // Non-POSIX: no serial

bool ServoDriver::openSerial() {
    log_.error()
        << "[ServoDriver] Serial not supported on this platform";
    return false;
}
void ServoDriver::closeSerial() {}

#endif // TORQ_SERIAL_POSIX

// ─── Connect / Disconnect ─────────────────────────────────

bool ServoDriver::connect(const std::string& config_path) {
    if (connected_) disconnect();
    if (!loadConfig(config_path)) return false;

    int n = static_cast<int>(config_.servo_ids.size());
    if (n == 0) {
        log_.error() << "[ServoDriver] No servo IDs configured";
        return false;
    }

    q_current_ = Eigen::VectorXd::Zero(n);
    qd_current_ = Eigen::VectorXd::Zero(n);
    ctrl_last_ = Eigen::VectorXd::Zero(n);

    if (!openSerial()) return false;

    sts_.setPort(serial_fd_);
    sts_.IOTimeOut = 500;  // ms (SDK default 100; allow slower buses)
    sts_.End = 0;          // little-endian (low byte first)

    connected_ = true;

    // Verify servos are reachable (using SCServo SDK)
    int found = 0;
    for (int i = 0; i < n; ++i) {
        if (pingServo(config_.servo_ids[i])) {
            ++found;
        } else {
            log_.warning() << "[ServoDriver] Servo ID "
                        << config_.servo_ids[i]
                        << " did not respond to PING";
        }
    }

    if (found == 0) {
        log_.error() << "[ServoDriver] No servos responded";
        disconnect();
        return false;
    }

    // Read initial positions (SDK handles sign-magnitude)
    readPositions();
    ctrl_last_ = q_current_;

    log_.info() << "[ServoDriver] Connected, "
                << found << "/" << n << " servos responding";
    return true;
}

void ServoDriver::disconnect() {
    if (!connected_) return;
    sts_.setPort(-1);  // stop using fd before close
    closeSerial();
    connected_ = false;
    log_.info() << "[ServoDriver] Disconnected";
}

// ─── Ping ─────────────────────────────────────────────────

bool ServoDriver::pingServo(int id) {
    int ret = sts_.Ping(static_cast<u8>(id));
    return (ret >= 0 && ret == id);
}

// ─── Unit Conversion ──────────────────────────────────────

double ServoDriver::rawToRadian(int raw, int /*index*/) const {
    double scale = (config_.position_scale > 0)
                       ? config_.position_scale
                       : defaultPositionScale();
    return (raw - config_.position_zero) * scale;
}

int ServoDriver::radianToRaw(double rad, int /*index*/) const {
    double scale = (config_.position_scale > 0)
                       ? config_.position_scale
                       : defaultPositionScale();
    int raw = static_cast<int>(
        std::round(config_.position_zero + rad / scale)
    );
    return std::clamp(raw, config_.position_min,
                      config_.position_max);
}

// ─── Read Positions (SCServo SDK; sign-magnitude handled by SDK) ───────────

bool ServoDriver::readPositions() {
    if (!connected_ || serial_fd_ < 0) return false;

    int n = static_cast<int>(config_.servo_ids.size());
    for (int i = 0; i < n; ++i) {
        int raw = sts_.ReadPos(config_.servo_ids[i]);
        if (raw < 0) {
            log_.warning() << "[ServoDriver] Read failed for ID "
                        << config_.servo_ids[i];
            continue;
        }
        double prev = q_current_(i);
        q_current_(i) = rawToRadian(raw, i);
        if (config_.control_period > 0) {
            qd_current_(i) =
                (q_current_(i) - prev) / config_.control_period;
        }
    }
    return true;
}

// ─── Read Positions via SYNC READ (SDK) ─────────────────────────

bool ServoDriver::readPositionsSyncRead() {
    if (!connected_ || serial_fd_ < 0) return false;

    int n = static_cast<int>(config_.servo_ids.size());
    if (n == 0 || n > 32) return false;

    u8 ids[32];
    for (int i = 0; i < n; ++i)
        ids[i] = static_cast<u8>(config_.servo_ids[i]);

    sts_.rFlushSCS();
    sts_.syncReadPacketTx(ids, static_cast<u8>(n), 56, 2);  // Present_Position_L, 2 bytes
    sts_.wFlushSCS();

    for (int i = 0; i < n; ++i) {
        u8 buf[2];
        if (sts_.syncReadPacketRx(ids[i], buf) != 2) {
            log_.warning() << "[ServoDriver] SyncRead: no reply from ID "
                        << config_.servo_ids[i];
            continue;
        }
        int raw = sts_.syncReadRxPacketToWrod(15);  // sign bit 15
        if (raw < 0) continue;
        double prev = q_current_(i);
        q_current_(i) = rawToRadian(raw, i);
        if (config_.control_period > 0) {
            qd_current_(i) =
                (q_current_(i) - prev) / config_.control_period;
        }
    }
    return true;
}

// ─── Write Positions via SYNC WRITE (SCServo SDK: SyncWritePosEx at ACC, 7 bytes) ───────────────────────

bool ServoDriver::writePositions(
    const Eigen::Ref<const Eigen::VectorXd>& q
) {
    if (!connected_ || serial_fd_ < 0) return false;

    int n = static_cast<int>(config_.servo_ids.size());
    if (q.size() != n || n > 32) return false;

    u8 ids[32];
    s16 positions[32];
    u16 speeds[32];
    u8 accs[32];

    for (int i = 0; i < n; ++i) {
        ids[i] = static_cast<u8>(config_.servo_ids[i]);
        positions[i] = static_cast<s16>(radianToRaw(q(i), i));
        speeds[i] = static_cast<u16>(config_.default_speed);
        accs[i] = 0;  // or config default if we add one
    }

    sts_.SyncWritePosEx(ids, static_cast<u8>(n), positions, speeds, accs);
    return true;
}

// ─── Write Single Servo (SDK WritePosEx; useful for debugging) ────────────

bool ServoDriver::writeSinglePosition(int id, double rad) {
    s16 pos = static_cast<s16>(radianToRaw(rad, 0));
    u16 speed = static_cast<u16>(config_.default_speed);
    return sts_.WritePosEx(static_cast<u8>(id), pos, speed, 0) != 0;
}

// ─── Public API ───────────────────────────────────────────

Eigen::VectorXd ServoDriver::getJointPositions() const {
    if (!connected_) return Eigen::VectorXd::Zero(0);
    return q_current_;
}

Eigen::VectorXd ServoDriver::getJointVelocities() const {
    if (!connected_)
        return Eigen::VectorXd::Zero(q_current_.size());
    return qd_current_;
}

Eigen::VectorXd ServoDriver::getCtrl() const {
    if (!connected_) return Eigen::VectorXd::Zero(0);
    return ctrl_last_;
}

void ServoDriver::setJointPositions(
    const Eigen::Ref<const Eigen::VectorXd>& q
) {
    if (!connected_) return;
    if (writePositions(q)) {
        ctrl_last_ = q;
    }
}

void ServoDriver::setJointVelocities(
    const Eigen::Ref<const Eigen::VectorXd>& qd
) {
    if (!connected_) return;
    int n = static_cast<int>(qd.size());
    if (n != static_cast<int>(ctrl_last_.size())) return;

    double dt = config_.control_period;
    Eigen::VectorXd q_new = ctrl_last_ + qd * dt;

    // Clamp to joint limits
    for (int i = 0; i < n; ++i) {
        double lo = rawToRadian(config_.position_min, i);
        double hi = rawToRadian(config_.position_max, i);
        q_new(i) = std::clamp(q_new(i), lo, hi);
    }

    setJointPositions(q_new);
}

void ServoDriver::step() {
    if (!connected_) return;
    readPositions();
}

double ServoDriver::getTimestep() const {
    return config_.control_period;
}

int ServoDriver::numJoints() const {
    return connected_
               ? static_cast<int>(config_.servo_ids.size())
               : 0;
}

int ServoDriver::numActuators() const {
    return numJoints();
}

// ─── Torque Enable/Disable (SDK + Lock/Unlock EPROM like LeRobot) ────────────────────────────────

bool ServoDriver::enableTorque(int id, bool enable) {
    int r = sts_.EnableTorque(static_cast<u8>(id), enable ? 1 : 0);
    if (r == 0) return false;
    if (!enable) {
        sts_.unLockEprom(static_cast<u8>(id));  // allow EPROM write when torque off
    } else {
        sts_.LockEprom(static_cast<u8>(id));
    }
    return true;
}

void ServoDriver::enableAllTorque(bool enable) {
    for (int id : config_.servo_ids) {
        sts_.EnableTorque(static_cast<u8>(id), enable ? 1 : 0);
        if (!enable)
            sts_.unLockEprom(static_cast<u8>(id));
        else
            sts_.LockEprom(static_cast<u8>(id));
    }
}

} // namespace torq