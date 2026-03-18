#ifndef TORQ_SCSERVO_SCS_PORT_HPP
#define TORQ_SCSERVO_SCS_PORT_HPP

/**
 * @file SCSPort.hpp
 * @brief Platform transport for SCServo (SC/STS/SMS) protocol — bottom layer of the servo stack.
 *
 * Implements the SCS virtual read/write interface using a POSIX serial file
 * descriptor so that the SCS protocol layer (INST.h, etc.) runs on Linux/macOS
 * without Arduino. The same pattern can be extended to ESP32 by implementing
 * writeSCS/readSCS with an ESP-IDF UART driver; ServoDriver and SMS_STS_Port
 * stay unchanged.
 *
 * **Usage:** setPort(fd) with an fd from open("/dev/ttyUSB0", O_RDWR) (or
 * equivalent), then use as the base for SMS_STS_Port (ST/STS/SMS series).
 *
 * @see SMS_STS_Port, ServoDriver
 */

#include "SCS.h"

namespace torq {
namespace scservo {

class SCSPort : public ::SCS {
public:
    SCSPort(u8 End = 0);
    /** Set serial fd (e.g. from open("/dev/ttyUSB0", O_RDWR)). -1 = disconnected. */
    void setPort(int fd);
    int getFd() const { return serial_fd_; }
    /** Timeout for readSCS in milliseconds (default 100). */
    unsigned long IOTimeOut;
    /** Last error from SDK (e.g. servo status byte). */
    int Err;

    int getErr() const { return Err; }

    /** Flush RX/TX (public so callers can use before sync read, etc.). */
    void rFlushSCS() override;
    void wFlushSCS() override;

protected:
    int writeSCS(unsigned char* nDat, int nLen) override;
    int readSCS(unsigned char* nDat, int nLen) override;
    int writeSCS(unsigned char bDat) override;

private:
    int serial_fd_;
};

} // namespace scservo
} // namespace torq

#endif
