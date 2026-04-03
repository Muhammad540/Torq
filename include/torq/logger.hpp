#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <mutex>

/** @brief ANSI colour escape codes for terminal output. */
namespace Color{
    const std::string RESET  = "\033[0m";
    const std::string GREEN  = "\033[32m";
    const std::string YELLOW = "\033[33m";
    const std::string RED    = "\033[31m";
}

/**
 * @brief Lightweight logger with coloured terminal output and optional file logging.
 *
 * Call Logger::enableFileLogging() once at program start to also write all
 * log messages (without ANSI colours) to a file in the current working
 * directory.
 *
 * Usage:
 * @code
 * Logger::enableFileLogging();          // writes to "log.txt" in cwd
 * Logger::enableFileLogging("run.log"); // custom filename
 *
 * Logger log;
 * log.info()    << "Loaded model with " << nq << " DOFs";
 * log.warning() << "Joint near limit";
 * log.error()   << "Failed to solve QP";
 * @endcode
 *
 * Each log stream flushes on destruction (terminal + file).
 * File logging is process-wide and thread-safe.
 */
class Logger{
    /** @brief Singleton state for file logging, shared across all Logger instances. */
    struct FileState {
        std::ofstream stream;
        std::mutex    mutex;
        bool          enabled = false;
    };

    static FileState& fileState() {
        static FileState s;
        return s;
    }

public:
    /**
     * @brief Enable writing log messages to a file in addition to the terminal.
     *
     * Opens (or truncates) the named file in the current working directory.
     * Should be called once at the start of main() before any log messages.
     *
     * @param filename  File name relative to the current working directory.
     *                  Defaults to "log.txt".
     */
    static void enableFileLogging(const std::string& filename = "log.txt") {
        FileState& fs = fileState();
        std::lock_guard<std::mutex> lock(fs.mutex);
        fs.stream.open(filename, std::ios::out | std::ios::trunc);
        fs.enabled = fs.stream.is_open();
        if (!fs.enabled) {
            std::cerr << "[Logger] Could not open log file: " << filename << std::endl;
        }
    }

    /**
     * @brief Flush and close the log file, disabling further file output.
     */
    static void disableFileLogging() {
        FileState& fs = fileState();
        std::lock_guard<std::mutex> lock(fs.mutex);
        fs.enabled = false;
        if (fs.stream.is_open()) fs.stream.close();
    }

    /** @brief True if file logging is currently active. */
    static bool isFileLoggingEnabled() {
        return fileState().enabled;
    }

    /**
     * @brief RAII stream that prints with an ANSI colour prefix on destruction.
     *
     * Collects output via `operator<<` and on destruction:
     *   - Prints to stdout with the ANSI colour prefix.
     *   - If file logging is enabled, writes the same message (without colour)
     *     to the log file, prefixed with the level label.
     */
    class LogStream {
        public:
            LogStream(const std::string& color, const std::string& level)
                : color_(color), level_(level) {}

            template <typename T>
            LogStream& operator<<(const T& value){
                buffer_ << value;
                return *this;
            }

            ~LogStream(){
                std::string msg = buffer_.str();
                std::cout << color_ << msg << Color::RESET << std::endl;

                FileState& fs = Logger::fileState();
                if (fs.enabled) {
                    std::lock_guard<std::mutex> lock(fs.mutex);
                    if (fs.stream.is_open()) {
                        fs.stream << "[" << level_ << "] " << msg << "\n";
                        fs.stream.flush();
                    }
                }
            }

        private:
            std::string        color_;
            std::string        level_;
            std::ostringstream buffer_;
    };

    /** @brief Green informational message. */
    LogStream info() {
        return LogStream(Color::GREEN, "INFO");
    }

    /** @brief Yellow warning message. */
    LogStream warning() {
        return LogStream(Color::YELLOW, "WARN");
    }

    /** @brief Red error message. */
    LogStream error() {
        return LogStream(Color::RED, "ERROR");
    }
};

#endif // LOGGER_HPP
