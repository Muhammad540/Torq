#ifndef TORQ_SERVO_DRIVER_HPP
#define TORQ_SERVO_DRIVER_HPP

/**
 * @file ServoDriver.hpp
 * @brief Real-hardware driver for ST/STS/SMS serial bus servos (e.g. ST3215).
 *
 * ## Architecture (bottom-up)
 *
 * The driver follows a layered design so the protocol and application logic
 * are independent of the transport:
 *
 * 1. **SCSPort** (scservo/SCSPort.hpp) — Platform transport implementing the
 *    SCS virtual read/write interface. The default implementation uses a
 *    POSIX serial file descriptor (Linux/macOS). This layer can be extended
 *    with an ESP32 UART implementation so the same SCS protocol and upper
 *    layers run on embedded.
 *
 * 2. **SMS_STS_Port** (scservo/SMS_STS_Port.hpp) — ST/STS/SMS series protocol
 *    on top of SCSPort: memory map (goal position, present position, torque
 *    enable, etc.), SyncWritePosEx, ReadPos, Ping, EnableTorque. Inherits
 *    from SCSPort; no Arduino dependency.
 *
 * 3. **ServoDriver** (this file) — Application-level HardwareInterface:
 *    loadConfig() (including .conf file parsing), openSerial() (POSIX when
 *    TORQ_SERIAL_POSIX), radian ↔ raw conversion, step()/readPositions()/
 *    writePositions(), and exposure of getJointPositions() / setJointPositions()
 *    for the control stack.
 *
 * ## Sim-to-real mapping
 *
 * Use the **same URDF and joint limits** as in simulation. The driver converts
 * between encoder raw (0–4095) and radians via position_zero and position_scale
 * so the rest of the stack (kinematics, IK, limits) sees SI units. Configure
 * VelocityLimit and ConfigurationLimit barriers so the IK never commands
 * motions outside safe ranges. The display mirror uses the same scene XML as
 * sim so the 3D model matches the real robot.
 *
 * ## Active vs passive control
 *
 * Control mode is decided by RobotConfig::active_control (set by the application,
 * e.g. from a .conf or CLI):
 *
 * - **Passive (active_control == false):** RobotSystem::update() calls
 *   hardware_->step() (so ServoDriver reads the bus and updates q_current_),
 *   then mirrors hardware_->getJointPositions() into the display MuJoCo model.
 *   controller_->update() is **not** called, so no position commands are sent.
 *   The user can move the arm by hand and see it in the GUI.
 *
 * - **Active (active_control == true):** After hardware_->step(), RobotSystem
 *   calls controller_->update(), which runs IK and calls
 *   hardware_->setJointPositions() with the result. The servos are commanded
 *   to track the task-space target.
 *
 * ServoDriver itself does not know about active/passive; it always reads in
 * step() and writes when setJointPositions() / setJointVelocities() are
 * invoked. The application chooses whether to invoke the controller.
 *
 * ## State mapping to GUI
 *
 * When using real hardware, RobotSystem can load a MuJoCo scene as a
 * display-only mirror (no physics step). Each update():
 *
 * 1. hardware_->step() — ServoDriver reads present positions from the bus
 *    into q_current_ (and updates qd_current_ by finite difference).
 * 2. RobotSystem calls display_physics_->overrideJointPositions(
 *      hardware_->getJointPositions() ) so the display model’s qpos matches
 *    the real robot.
 * 3. The GUI renders the display_physics_ scene; the user sees the real
 *    arm pose in 3D.
 *
 * So the state path is: bus → readPositions() → q_current_ →
 * getJointPositions() → overrideJointPositions() → MuJoCo qpos → GUI.
 *
 * ## Config structure and how it is passed and parsed
 *
 * **Passing:** The application sets RobotConfig::driver_connection to either
 * a serial port path (e.g. `/dev/ttyUSB0`) or a config file path (e.g.
 * `workspace/so101/so101.conf`). RobotSystem::initialize() passes that string
 * to HardwareInterface::connect(); for ServoDriver, connect(config_path) calls
 * loadConfig(config_path) then openSerial() using the resolved port.
 *
 * **Parsing (loadConfig):**
 * - If config_path contains `.json`, `.yaml`, or `.conf`, it is opened as a
 *   text file and parsed as **key=value** lines (one key per line). Lines
 *   without `=` are ignored (so comments and blanks are effectively skipped).
 *   No trimming of key names; values are taken as the substring after the
 *   first `=`.
 * - If the path does not look like a config file, it is used as the serial
 *   port path and ST3215 defaults apply (baud 1000000, control_period 0.002,
 *   servo_ids 1..6, etc.).
 *
 * **Keys recognized** (all optional when using a config file):
 * | Key              | Type   | Effect                          | Default (file or raw port) |
 * |------------------|--------|----------------------------------|-----------------------------|
 * | port             | string | Serial device path               | config_path (if not file)   |
 * | baud_rate        | int    | Baud rate                        | 1000000                     |
 * | protocol         | string | Protocol label                   | "st3215"                    |
 * | control_period   | double | Timestep [s] for step()/getTimestep() | 0.002                |
 * | default_speed    | int    | Speed used in SYNC WRITE          | 2000                        |
 * | servo_ids        | string | Space-separated IDs, joint order | 1 2 3 4 5 6                 |
 *
 * After parsing, if servo_ids is empty it is set to 1..6. Other
 * SerialServoConfig fields (position_min, position_max, position_scale,
 * position_zero, default_time) are not read from the file; they stay at
 * ST3215 defaults. Use config() after connect() to inspect the resolved config.
 */

#include "torq/HardwareInterface.hpp"
#include "torq/logger.hpp"
#include "torq/scservo/SMS_STS_Port.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace torq {

    /**
     * @brief Configuration for a serial bus servo driver (ST3215).
     *
     * ST3215 specs:
     *   - 12-bit magnetic encoder, 0-4095 raw range, 360° rotation
     *   - RS485 half-duplex bus, 1 Mbps default baud
     *   - Low byte first, high byte last (little-endian)
     *   - Write block at 0x2A: position(2) + time(2) + speed(2) = 6 bytes
     *   - Voltage range: 6-12V
     */
    struct SerialServoConfig {
        /** Serial port path: "/dev/ttyUSB0", "COM3", etc. */
        std::string port;

        /** Baud rate (ST3215 default: 1000000). */
        int baud_rate = 1000000;

        /** Servo IDs in joint order (daisy-chain order). */
        std::vector<int> servo_ids;

        /** Protocol identifier. */
        std::string protocol = "st3215";

        /**
         * Control period [s] for step() and getTimestep().
         * e.g. 0.002 = 500 Hz.
         */
        double control_period = 0.002;

        /** Raw position range (ST3215: 0-4095, 12-bit). */
        int position_min = 0;
        int position_max = 4095;

        /**
         * Scale: position_rad = (raw - position_zero) * position_scale.
         * 0 = use default (2*PI / 4096). Same convention as simulation so
         * URDF joint limits and IK apply unchanged.
         */
        double position_scale = 0.0;

        /** Raw encoder value corresponding to 0 radians (ST3215 centre: 2047.5). */
        double position_zero = 2047.5;

        /**
         * Default movement speed (ST3215 units, max 4000).
         * Used in SYNC WRITE / WRITE position commands.
         */
        int default_speed = 2000;

        /**
         * Default movement time (ST3215 units, 0 = speed-controlled).
         * When non-zero, servo interpolates over this duration.
         */
        int default_time = 0;
    };

    /**
     * @brief Hardware driver for ST/STS/SMS serial bus servos (e.g. ST3215).
     *
     * Implements HardwareInterface for RS485 servos using the SCServo-based
     * stack: SCSPort (transport) → SMS_STS_Port (protocol) → this class
     * (config, radian conversion, step/read/write). Uses SYNC WRITE for
     * commanding multiple servos in one bus transaction and READ (or SYNC READ)
     * for position feedback.
     *
     * **Protocol (ST3215 / STS):**
     *   - Packet: [0xFF][0xFF][ID][LEN][INST][params...][CHK]
     *   - Checksum: ~(ID + LEN + INST + params) & 0xFF
     *   - Byte order: little-endian (low byte first); sign-magnitude for
     *     position (bit 15 = sign, handled by SMS_STS_Port).
     *   - Bus: RS485 half-duplex, typically 1 Mbps
     *
     * **Config:** connect() accepts a serial port path (e.g. `/dev/ttyUSB0`)
     * or a config file path. If the string contains `.json`, `.yaml`, or
     * `.conf`, it is parsed as key=value; keys: port, baud_rate, protocol,
     * control_period, default_speed, servo_ids (space-separated). Otherwise
     * the string is used as the port and ST3215 defaults apply (baud 1000000,
     * IDs 1..6).
     *
     * **State:** getJointPositions() and getJointVelocities() return the
     * cached values from the last successful step() / readPositions().
     * Velocities are estimated by finite difference over control_period.
     *
     * @see HardwareInterface, MujocoDriver, RobotConfig::active_control,
     *      RobotConfig::driver_connection, SCSPort, SMS_STS_Port
     */
    class ServoDriver : public HardwareInterface {
    public:
        ServoDriver();
        ~ServoDriver() override;

        ServoDriver(const ServoDriver&) = delete;
        ServoDriver& operator=(const ServoDriver&) = delete;

        /**
         * @brief Open connection to servos.
         *
         * If config_path contains `.json`, `.yaml`, or `.conf`, it is opened
         * as a config file (key=value, one per line). Keys: port, baud_rate,
         * protocol, control_period, default_speed, servo_ids. Otherwise
         * config_path is used as the serial port path with ST3215 defaults
         * (baud 1000000, IDs 1..6).
         *
         * After opening the port and attaching the SCS transport, pings all
         * configured servo IDs and reads initial positions into q_current_
         * and ctrl_last_. Fails if no servos respond.
         */
        bool connect(const std::string& config_path) override;
        void disconnect() override;

        /**
         * @brief Current joint positions [rad] (cached from last step() / readPositions()).
         *
         * Used by RobotSystem for kinematics and for display mirror
         * (overrideJointPositions() so the GUI shows the real arm pose).
         */
        Eigen::VectorXd getJointPositions() const override;

        /**
         * @brief Get numerically estimated joint velocities.
         *
         * Computed as finite difference of position readings.
         */
        Eigen::VectorXd getJointVelocities() const override;

        /** @brief Get the last commanded position vector. */
        Eigen::VectorXd getCtrl() const override;

        /**
         * @brief Command joint positions via SYNC WRITE.
         *
         * Writes position + time + speed block to all servos
         * atomically in a single bus packet.
         */
        void setJointPositions(
            const Eigen::Ref<const Eigen::VectorXd>& q
        ) override;

        /**
         * @brief Command joint velocities via position integration.
         *
         * Integrates: q_new = q_last + qd * dt, clamps to joint
         * limits, then calls setJointPositions().
         */
        void setJointVelocities(
            const Eigen::Ref<const Eigen::VectorXd>& qd
        ) override;

        /**
         * @brief Read current positions from the bus and update cached state.
         *
         * Calls readPositions() (per-servo READ or SYNC READ), which updates
         * q_current_ and qd_current_. Called every update() by RobotSystem;
         * in passive mode this is the only hardware write/read — no
         * setJointPositions() is invoked, so the arm can be moved by hand
         * and the GUI reflects the read-back pose.
         */
        void step() override;

        double getTimestep() const override;
        int numJoints() const override;
        int numActuators() const override;

        /** @brief Whether the driver is connected. */
        bool isConnected() const { return connected_; }

        /** @brief Current config (valid after connect()). */
        const SerialServoConfig& config() const { return config_; }

        // ── Additional hardware control ─────────────────────

        /**
         * @brief Ping a servo to check if it's alive.
         * @param id Servo ID (0-253).
         * @return true if servo responded without error.
         */
        bool pingServo(int id);

        /**
         * @brief Enable/disable torque on a single servo.
         * @param id Servo ID.
         * @param enable true to lock, false to release (free-spin).
         */
        bool enableTorque(int id, bool enable);

        /**
         * @brief Enable/disable torque on all configured servos.
         */
        void enableAllTorque(bool enable);

        /**
         * @brief Write a single servo to a position (uses WRITE,
         *        not SYNC WRITE). Useful for debugging.
         * @param id Servo ID.
         * @param rad Target position in radians.
         */
        bool writeSinglePosition(int id, double rad);

        /**
         * @brief Read positions using SYNC READ (0x82).
         *
         * More efficient than individual reads when many servos
         * are on the bus. Requires servo firmware support.
         */
        bool readPositionsSyncRead();

    private:
        // ── Config & serial ─────────────────────────────────
        /** Load config from path (parse .json/.yaml/.conf) or use path as port. */
        bool loadConfig(const std::string& path_or_port);
        bool openSerial();
        void closeSerial();

        // ── Position I/O (via SCServo SDK) ────────────────────
        bool readPositions();
        bool writePositions(
            const Eigen::Ref<const Eigen::VectorXd>& q
        );

        // ── Unit conversion ─────────────────────────────────

        /**
         * @brief Convert raw encoder value to radians.
         *
         * rad = (raw - position_zero) * position_scale
         */
        double rawToRadian(int raw, int index) const;

        /**
         * @brief Convert radians to raw encoder value (clamped).
         *
         * raw = clamp(round(position_zero + rad / position_scale),
         *             position_min, position_max)
         */
        int radianToRaw(double rad, int index) const;

        // ── State ───────────────────────────────────────────
        SerialServoConfig config_;
        bool connected_ = false;
        int serial_fd_ = -1;

        /** ST/STS/SMS protocol*/
        scservo::SMS_STS_Port sts_;

        Eigen::VectorXd q_current_;   ///< Latest read positions [rad]; source for getJointPositions() and GUI mirror
        Eigen::VectorXd qd_current_;  ///< Estimated velocities [rad/s] from finite difference
        Eigen::VectorXd ctrl_last_;   ///< Last commanded positions [rad]; source for getCtrl()

        Logger log_;
    };

} // namespace torq

#endif // TORQ_SERVO_DRIVER_HPP