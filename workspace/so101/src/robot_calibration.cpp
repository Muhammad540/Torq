/**
 * Calibration tool for SO101 arm.
 *
 * Step 1: Init scan servos, torque off, unlock EEPROM, reset homing offset.
 * Step 2: Hardware centering (flashes Homing Offset so center is 2048)
 * Step 3: Hardware limits (flashes Min/Max physical limits)
 * Step 4: Lock EEPROM, write calibration file (raw_center=2048, raw_min/raw_max from sweep).
 *         Defaults all directions to +1; flip manually in the file if needed.
 *
 * Usage:
 *   ./calibrate [port] [output_path]
 */

 #include "torq/scservo/SMS_STS_Port.hpp"
 
 #include <algorithm>
 #include <chrono>
 #include <cmath>
 #include <ctime>
 #include <filesystem>
 #include <fstream>
 #include <iomanip>
 #include <iostream>
 #include <string>
 #include <thread>
 #include <vector>
 
 #include <fcntl.h>
 #include <termios.h>
 #include <unistd.h>
 #include <poll.h>
 
 namespace Color {
     const char* Reset   = "\033[0m";
     const char* Bold    = "\033[1m";
     const char* Red     = "\033[1;31m";
     const char* Green   = "\033[1;32m";
     const char* Yellow  = "\033[1;33m";
     const char* Blue    = "\033[1;34m";
     const char* Magenta = "\033[1;35m";
     const char* Cyan    = "\033[1;36m";
 }
 
 constexpr u8 STS_MIN_ANGLE_LIMIT_L = 9;
 constexpr u8 STS_MAX_ANGLE_LIMIT_L = 11;
 constexpr u8 STS_HOMING_OFFSET_L   = 31;
 
 struct JointSpec {
     std::string name;
     double mj_low;
     double mj_high;
 };
 
 static const std::vector<JointSpec> SO101_JOINTS = {
     {"shoulder_pan",  -1.91986, 1.91986},
     {"shoulder_lift", -1.74533, 1.74533},
     {"elbow_flex",    -1.69,    1.69   },
     {"wrist_flex",    -1.65806, 1.65806},
     {"wrist_roll",    -2.74385, 2.84121},
     {"gripper",       -0.17453, 1.74533},
 };
 
 static int openPort(const std::string& port, int baud_rate) {
     int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
     if (fd < 0) {
         std::cerr << Color::Red << "  ERROR: Cannot open " << port << Color::Reset << "\n";
         return -1;
     }
     fcntl(fd, F_SETFL, 0);
 
     struct termios tty{};
     tcgetattr(fd, &tty);
 
     speed_t speed;
     switch (baud_rate) {
         case 57600:   speed = B57600;   break;
         case 115200:  speed = B115200;  break;
         case 500000:  speed = B500000;  break;
         case 1000000: speed = B1000000; break;
         default:
             std::cerr << Color::Red << "  ERROR: Unsupported baud: " << baud_rate << Color::Reset << "\n";
             close(fd);
             return -1;
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
 
     tcsetattr(fd, TCSANOW, &tty);
     tcflush(fd, TCIOFLUSH);
     return fd;
 }
 
 /// Encode a signed homing offset into Feetech sign-magnitude format.
 /// Bit 11 = sign (1 = negative), bits 0..10 = magnitude. Max ±2047.
 static u16 encodeHomingOffset(int offset) {
     int magnitude = std::min(std::abs(offset), 2047);
     u16 encoded = static_cast<u16>(magnitude);
     if (offset < 0) encoded |= (1u << 11);
     return encoded;
 }
 
 static void waitForEnter() {
     std::string dummy;
     std::getline(std::cin, dummy);
 }
 
 int main(int argc, char** argv) {
     std::filesystem::path root(PROJECT_ROOT);
 
     std::string port = "/dev/ttyACM0";
     std::string output_path = (root / "workspace/so101/calibration/calibration.txt").string();
     int baud_rate = 1000000;
 
     if (argc >= 2) port = argv[1];
     if (argc >= 3) output_path = argv[2];
 
     int num_joints = static_cast<int>(SO101_JOINTS.size());
 
     std::cout << Color::Cyan 
               << "\n"
               << "  ==========================================\n"
               << "             SO101 Calibration Tool         \n"
               << "  ==========================================\n"
               << "  Port:   " << port << "\n"
               << "  Output: " << output_path << "\n\n" 
               << Color::Reset;
 
     int fd = openPort(port, baud_rate);
     if (fd < 0) return 1;
 
     torq::scservo::SMS_STS_Port sts;
     sts.setPort(fd);
     sts.IOTimeOut = 500;
     sts.End = 0;

     std::cout << Color::Yellow 
               << "  STEP 1: Initialization\n"
               << "  Scanning for servos...\n";
 
     std::vector<int> found_ids;
     for (int id = 1; id <= num_joints; ++id) {
         int ret = sts.Ping(static_cast<u8>(id));
         if (ret >= 0 && ret == id) {
             found_ids.push_back(id);
             std::cout << "    Found servo ID " << id << "\n";
         }
     }
 
     if (static_cast<int>(found_ids.size()) != num_joints) {
         std::cerr << Color::Red << "  ERROR: Expected " << num_joints << " servos, found "
                   << found_ids.size() << ". Aborting.\n" << Color::Reset;
         close(fd);
         return 1;
     }
 
     std::cout << "  Disabling torque and unlocking EEPROM...\n";
     for (int id : found_ids) {
         sts.EnableTorque(static_cast<u8>(id), 0);
         sts.unLockEprom(static_cast<u8>(id));
     }
 
     std::cout << "  Resetting homing offsets to 0...\n";
     for (int id : found_ids) {
         sts.writeWord(static_cast<u8>(id), STS_HOMING_OFFSET_L, 0);
     }
     std::this_thread::sleep_for(std::chrono::milliseconds(200));
 
     std::cout << "  Step 1 complete.\n\n" << Color::Reset;
 
     std::cout << Color::Green 
               << "  STEP 2: Hardware centering\n"
               << "  Move ALL joints to roughly MIDDLE of their range.\n"
               << "  Press ENTER when ready.\n";
 
     waitForEnter();
 
     std::cout << "  Reading positions and computing homing offsets...\n";
     bool centering_ok = true;
     for (int i = 0; i < num_joints; ++i) {
         int id = found_ids[i];
         int raw = sts.ReadPos(id);
         if (raw < 0) {
             std::cerr << Color::Red << "  ERROR: Cannot read servo " << id << ". Aborting.\n" << Color::Reset;
             close(fd);
             return 1;
         }
 
         int offset = raw - 2048;
         u16 encoded = encodeHomingOffset(offset);
         sts.writeWord(static_cast<u8>(id), STS_HOMING_OFFSET_L, encoded);
 
         std::cout << "    " << std::left << std::setw(15) << SO101_JOINTS[i].name
                   << "raw=" << raw << "  offset=" << offset
                   << "  encoded=0x" << std::hex << encoded << std::dec << "\n";
     }
 
     std::this_thread::sleep_for(std::chrono::milliseconds(200));
 
     std::cout << "  Verifying — all servos should now read roughly 2048...\n";
     for (int i = 0; i < num_joints; ++i) {
         int id = found_ids[i];
         int raw = sts.ReadPos(id);
         int err = std::abs(raw - 2048);
         const char* status = (err <= 10) ? "OK" : "FAIL";
         std::cout << "    " << std::left << std::setw(15) << SO101_JOINTS[i].name
                   << "reads " << raw << "  " << status << "\n";
         if (err > 10) centering_ok = false;
     }
 
     if (!centering_ok) {
         std::cerr << Color::Red << "  ERROR: Homing offset verification failed. Please restart calibration.\n" << Color::Reset;
         close(fd);
         return 1;
     }
     std::cout << "  Step 2 complete. Homing offsets flashed.\n\n" << Color::Reset;
 
     std::cout << Color::Magenta 
               << "  STEP 3: Hardware limits\n"
               << "  Move ALL joints through their FULL range of motion.\n"
               << "  Push each joint to BOTH hard stops.\n"
               << "  (You can move them in any order, all are tracked. Recommended is to start from shoulder_pan and move to gripper)\n"
               << "  Press ENTER when done.\n";
 
     std::vector<int> track_min(num_joints, 4095);
     std::vector<int> track_max(num_joints, 0);
 
     for (int i = 0; i < num_joints; ++i) {
         int raw = sts.ReadPos(found_ids[i]);
         if (raw >= 0) {
             track_min[i] = raw;
             track_max[i] = raw;
         }
     }
 
     struct termios orig_term, raw_term;
     tcgetattr(STDIN_FILENO, &orig_term);
     raw_term = orig_term;
     raw_term.c_lflag &= ~(ICANON | ECHO);
     raw_term.c_cc[VMIN] = 0;
     raw_term.c_cc[VTIME] = 0;
     tcsetattr(STDIN_FILENO, TCSANOW, &raw_term);
 
     bool limits_done = false;
     while (!limits_done) {
         for (int i = 0; i < num_joints; ++i) {
             int raw = sts.ReadPos(found_ids[i]);
             if (raw >= 0) {
                 track_min[i] = std::min(track_min[i], raw);
                 track_max[i] = std::max(track_max[i], raw);
             }
         }
 
         std::cout << "\r  ";
         for (int i = 0; i < num_joints; ++i) {
             std::cout << SO101_JOINTS[i].name.substr(0, 6) << "=["
                       << track_min[i] << "," << track_max[i] << "] ";
         }
         std::cout << std::flush;
 
         char ch;
         if (read(STDIN_FILENO, &ch, 1) > 0 && (ch == '\n' || ch == '\r'))
             limits_done = true;
 
         if (!limits_done)
             std::this_thread::sleep_for(std::chrono::milliseconds(30));
     }
 
     tcsetattr(STDIN_FILENO, TCSANOW, &orig_term);
 
     std::cout << "\n  Writing EEPROM min/max limits...\n";
     for (int i = 0; i < num_joints; ++i) {
         int id = found_ids[i];
         sts.writeWord(static_cast<u8>(id), STS_MIN_ANGLE_LIMIT_L,
                       static_cast<u16>(track_min[i]));
         sts.writeWord(static_cast<u8>(id), STS_MAX_ANGLE_LIMIT_L,
                       static_cast<u16>(track_max[i]));
         std::cout << "    " << std::left << std::setw(15) << SO101_JOINTS[i].name
                   << "min=" << track_min[i] << "  max=" << track_max[i] << "\n";
     }
     std::cout << "  Step 3 complete. EEPROM limits flashed.\n\n" << Color::Reset;
 
     std::cout << Color::Blue 
               << "  STEP 4: Locking EEPROM and saving config\n";
 
     for (int id : found_ids)
         sts.LockEprom(static_cast<u8>(id));
 
     std::filesystem::path out_dir = std::filesystem::path(output_path).parent_path();
     if (!out_dir.empty())
         std::filesystem::create_directories(out_dir);
 
     std::ofstream out(output_path);
     if (!out.is_open()) {
         std::cerr << Color::Red << "  ERROR: Cannot write to " << output_path << Color::Reset << "\n";
         close(fd);
         return 1;
     }
 
     auto now = std::time(nullptr);
     char time_buf[64];
     std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%dT%H:%M:%S",
                   std::localtime(&now));
 
     out << "# SO101 Calibration — generated " << time_buf << "\n"
         << "#\n"
         << "port=" << port << "\n"
         << "baud_rate=" << baud_rate << "\n"
         << "#\n"
         << "# Directions default to +1. Modify manually if runtime simulation is flipped.\n"
         << "# joint_index servo_id joint_name direction raw_center raw_min raw_max\n"
         << "# ServoDriver: center anchor for all joints; mj_low anchor only for gripper.\n";
 
     for (int i = 0; i < num_joints; ++i) {
         out << i << " "
             << found_ids[i] << " "
             << SO101_JOINTS[i].name << " "
             << 1 << " "
             << 2048 << " "
             << track_min[i] << " "
             << track_max[i] << "\n";
     }
     out.close();
 
     close(fd);
 
     std::cout << "\n  Calibration saved to: " << output_path << "\n"
               << "  EEPROM is locked. Run passive mode, and manually flip directions in file if needed.\n\n" 
               << Color::Reset;
 
     return 0;
 }