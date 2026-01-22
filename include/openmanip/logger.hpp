#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

namespace Color{
    const std::string RESET  = "\033[0m";
    const std::string GREEN  = "\033[32m";
    const std::string YELLOW = "\033[33m";
    const std::string RED    = "\033[31m";
}

class Logger{
    public:
        class LogStream {
            private:
                std::string color_;
                std::ostringstream buffer_;
            public:
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

        LogStream info() {
            return LogStream(Color::GREEN);
        }
    
        LogStream warning() {
            return LogStream(Color::YELLOW);
        }
    
        LogStream error() {
            return LogStream(Color::RED);
        }
};

#endif // LOGGER_HPP