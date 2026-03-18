/*
 * SMS_STS_Port.cpp
 * ST/STS/SMS series on SCSPort (logic from SCServo SDK SMS_STS).
 */

#include "torq/scservo/SMS_STS_Port.hpp"

namespace torq {
namespace scservo {

SMS_STS_Port::SMS_STS_Port() : SCSPort(0) {}

SMS_STS_Port::SMS_STS_Port(u8 End) : SCSPort(End) {}

int SMS_STS_Port::WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC) {
    if (Position < 0) {
        Position = -Position;
        Position |= (1 << 15);
    }
    u8 bBuf[7];
    bBuf[0] = ACC;
    Host2SCS(bBuf + 1, bBuf + 2, static_cast<u16>(Position));
    Host2SCS(bBuf + 3, bBuf + 4, 0);
    Host2SCS(bBuf + 5, bBuf + 6, Speed);
    return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

void SMS_STS_Port::SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]) {
    u8 offbuf[7 * 32];  // max 32 servos per packet
    if (IDN > 32) return;
    for (u8 i = 0; i < IDN; i++) {
        s16 P = Position[i];
        if (P < 0) {
            P = -P;
            P |= (1 << 15);
        }
        u16 V = Speed ? Speed[i] : 0;
        offbuf[i * 7] = ACC ? ACC[i] : 0;
        Host2SCS(offbuf + i * 7 + 1, offbuf + i * 7 + 2, static_cast<u16>(P));
        Host2SCS(offbuf + i * 7 + 3, offbuf + i * 7 + 4, 0);
        Host2SCS(offbuf + i * 7 + 5, offbuf + i * 7 + 6, V);
    }
    syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
}

int SMS_STS_Port::EnableTorque(u8 ID, u8 Enable) {
    return writeByte(ID, SMS_STS_TORQUE_ENABLE, Enable);
}

int SMS_STS_Port::unLockEprom(u8 ID) {
    return writeByte(ID, SMS_STS_LOCK, 0);
}

int SMS_STS_Port::LockEprom(u8 ID) {
    return writeByte(ID, SMS_STS_LOCK, 1);
}

int SMS_STS_Port::FeedBack(int ID) {
    int nLen = Read(ID, SMS_STS_PRESENT_POSITION_L, Mem, sizeof(Mem));
    if (nLen != static_cast<int>(sizeof(Mem))) {
        Err = 1;
        return -1;
    }
    Err = 0;
    return nLen;
}

int SMS_STS_Port::ReadPos(int ID) {
    int Pos = -1;
    if (ID == -1) {
        Pos = Mem[SMS_STS_PRESENT_POSITION_H - SMS_STS_PRESENT_POSITION_L];
        Pos <<= 8;
        Pos |= Mem[SMS_STS_PRESENT_POSITION_L - SMS_STS_PRESENT_POSITION_L];
    } else {
        Err = 0;
        Pos = readWord(ID, SMS_STS_PRESENT_POSITION_L);
        if (Pos == -1)
            Err = 1;
    }
    if (!Err && (Pos & (1 << 15)))
        Pos = -(Pos & ~(1 << 15));
    return Pos;
}

int SMS_STS_Port::ReadSpeed(int ID) {
    int Speed = -1;
    if (ID == -1) {
        Speed = Mem[SMS_STS_PRESENT_SPEED_H - SMS_STS_PRESENT_POSITION_L];
        Speed <<= 8;
        Speed |= Mem[SMS_STS_PRESENT_SPEED_L - SMS_STS_PRESENT_POSITION_L];
    } else {
        Err = 0;
        Speed = readWord(ID, SMS_STS_PRESENT_SPEED_L);
        if (Speed == -1) {
            Err = 1;
            return -1;
        }
    }
    if (!Err && (Speed & (1 << 15)))
        Speed = -(Speed & ~(1 << 15));
    return Speed;
}

int SMS_STS_Port::Ping(u8 ID) {
    return SCS::Ping(ID);
}

} // namespace scservo
} // namespace torq
