#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

/** @brief ANSI colour escape codes for terminal output. */
namespace Color{
    const std::string RESET  = "\033[0m";
    const std::string GREEN  = "\033[32m";
    const std::string YELLOW = "\033[33m";
    const std::string RED    = "\033[31m";
}

/**
 * @brief Lightweight logger with coloured terminal output.
 *
 * Usage:
 * @code
 * Logger log;
 * log.info()    << "Loaded model with " << nq << " DOFs";
 * log.warning() << "Joint near limit";
 * log.error()   << "Failed to solve QP";
 * @endcode
 *
 * Each log stream flushes (with a newline and colour reset) on destruction.
 */
class Logger{
    public:
        /**
         * @brief RAII stream that prints with an ANSI colour prefix.
         *
         * Collects output via `operator<<` and prints the accumulated message
         * in the specified colour on destruction.
         */
        class LogStream {
            private:
                std::string color_;
                std::ostringstream buffer_;
            public:
                /** @brief Construct with an ANSI colour code. */
                LogStream(const std::string& color): color_(color){}
                
                template <typename T>
                LogStream& operator<<(const T& value){
                    buffer_ << value;
                    return *this;
                }
                
                ~LogStream(){
                    std::cout << color_ << buffer_.str() << Color::RESET << std::endl;
                }
        };

        /** @brief Green informational message. */
        LogStream info() {
            return LogStream(Color::GREEN);
        }
    
        /** @brief Yellow warning message. */
        LogStream warning() {
            return LogStream(Color::YELLOW);
        }
    
        /** @brief Red error message. */
        LogStream error() {
            return LogStream(Color::RED);
        }
};

#endif // LOGGER_HPP
