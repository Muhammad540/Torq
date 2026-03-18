#ifndef TORQ_SCSERVO_SMS_STS_PORT_HPP
#define TORQ_SCSERVO_SMS_STS_PORT_HPP

/**
 * @file SMS_STS_Port.hpp
 * @brief ST/STS/SMS series protocol on top of SCSPort — middle layer of the servo stack.
 *
 * Same memory map and protocol as the SCServo SDK SMS_STS; the base is our
 * SCSPort (POSIX or extensible to ESP32), so no Arduino dependency. Provides
 * WritePosEx, SyncWritePosEx, ReadPos, Ping, EnableTorque, and sync read
 * support. ServoDriver uses this for all bus I/O and radian conversion above.
 *
 * @see SCSPort, ServoDriver
 */

#include "torq/scservo/SCSPort.hpp"
#include "INST.h"

namespace torq {
namespace scservo {

// Memory map
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_LOCK 55
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70

class SMS_STS_Port : public ::torq::scservo::SCSPort {
public:
    SMS_STS_Port();
    explicit SMS_STS_Port(u8 End);

    /** Write position + speed + acc (single servo). */
    int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);
    /** Sync write position + speed + acc for multiple servos. */
    void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);
    /** Enable/disable torque. */
    int EnableTorque(u8 ID, u8 Enable);
    /** Unlock EPROM (e.g. before calibration). */
    int unLockEprom(u8 ID);
    /** Lock EPROM. */
    int LockEprom(u8 ID);
    /** Read present position (sign-magnitude bit 15). ID=-1 uses cached Mem from FeedBack; -1 return = error. */
    int ReadPos(int ID);
    /** Read present speed. ID=-1 uses cached Mem; -1 return = error. */
    int ReadSpeed(int ID);
    /** Read multiple registers from present position (for FeedBack). */
    int FeedBack(int ID);
    /** Ping; returns ID on success, -1 on timeout. */
    int Ping(u8 ID);

private:
    u8 Mem[SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L + 1];
};

} // namespace scservo
} // namespace torq

#endif
