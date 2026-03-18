#include "torq/scservo/SCSPort.hpp"

#include <chrono>
#include <cstring>
#include <cerrno>

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#else
#error "SCSPort requires POSIX (Linux/macOS) or ESP32"
#endif

namespace torq {
namespace scservo {

static unsigned long millis() {
    using Clock = std::chrono::steady_clock;
    static auto start = Clock::now();
    return static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count());
}

SCSPort::SCSPort(u8 End) : SCS(End), IOTimeOut(100), Err(0), serial_fd_(-1) {}

void SCSPort::setPort(int fd) {
    serial_fd_ = fd;
}

void SCSPort::rFlushSCS() {
#if defined(__linux__) || defined(__APPLE__)
    if (serial_fd_ >= 0) {
        tcflush(serial_fd_, TCIFLUSH);
    }
#endif
}

void SCSPort::wFlushSCS() {
#if defined(__linux__) || defined(__APPLE__)
    if (serial_fd_ >= 0) {
        tcdrain(serial_fd_);
    }
#endif
}

int SCSPort::writeSCS(unsigned char* nDat, int nLen) {
#if defined(__linux__) || defined(__APPLE__)
    if (serial_fd_ < 0 || !nDat) return 0;
    ssize_t n = write(serial_fd_, nDat, static_cast<size_t>(nLen));
    return (n >= 0 && n == static_cast<ssize_t>(nLen)) ? static_cast<int>(n) : 0;
#else
    (void)nDat;
    (void)nLen;
    return 0;
#endif
}

int SCSPort::writeSCS(unsigned char bDat) {
#if defined(__linux__) || defined(__APPLE__)
    if (serial_fd_ < 0) return 0;
    ssize_t n = write(serial_fd_, &bDat, 1);
    return (n == 1) ? 1 : 0;
#else
    (void)bDat;
    return 0;
#endif
}

int SCSPort::readSCS(unsigned char* nDat, int nLen) {
#if defined(__linux__) || defined(__APPLE__)
    if (serial_fd_ < 0) return 0;
    unsigned long t_begin = millis();
    int Size = 0;
    while (Size < nLen) {
        fd_set rfds;
        struct timeval tv;
        unsigned long t_elapsed = millis() - t_begin;
        if (t_elapsed > IOTimeOut)
            break;
        tv.tv_sec = 0;
        tv.tv_usec = 1000 * (IOTimeOut - static_cast<unsigned int>(t_elapsed));
        if (tv.tv_usec > 1000000) tv.tv_usec = 1000000;
        FD_ZERO(&rfds);
        FD_SET(serial_fd_, &rfds);
        int ret = select(serial_fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret <= 0) break;
        ssize_t n = read(serial_fd_, nDat + Size, static_cast<size_t>(nLen - Size));
        if (n <= 0) break;
        Size += static_cast<int>(n);
        t_begin = millis();
    }
    return Size;
#else
    (void)nDat;
    (void)nLen;
    return 0;
#endif
}

} // namespace scservo
} // namespace torq
