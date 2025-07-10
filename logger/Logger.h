// Logger.h
#ifndef LOGGER_H
#define LOGGER_H

#pragma once
#include <iostream>
#include <string>
#include <mutex>
#include <chrono>
#include <iomanip> // For std::put_time
#include <sstream> // For std::ostringstream to build messages

// Forward declaration if RDT::LogLevel is defined elsewhere,
// but for a standalone logger, it's fine to define it here.
// namespace RDT { enum class LogLevel; }

namespace RDT { // Assuming Logger is part of the RDT utilities

enum class LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    Critical,
    None // To disable logging
};

class Logger {
public:
    /**
     * @brief Sets the minimum log level to output.
     * Messages with a level lower than this will be ignored.
     * @param level The minimum LogLevel.
     */
    static void setLogLevel(LogLevel level) {
        get().current_level_ = level;
    }

    /**
     * @brief Logs a message if its level is sufficient.
     * @param level The LogLevel of the message.
     * @param module The name of the module/component logging the message.
     * @param message The message string to log.
     */
    static void log(LogLevel level, const std::string& module, const std::string& message) {
        Logger& instance = get();
        if (level >= instance.current_level_ && instance.current_level_ != LogLevel::None) {
            std::lock_guard<std::mutex> lock(instance.log_mutex_);
            
            // Get current time
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

            std::cout << "[" << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
            std::cout << '.' << std::setfill('0') << std::setw(3) << ms.count() << "]";
            
            switch (level) {
                case LogLevel::Debug:    std::cout << "[DBG]"; break;
                case LogLevel::Info:     std::cout << "[INF]"; break;
                case LogLevel::Warning:  std::cout << "[WRN]"; break;
                case LogLevel::Error:    std::cout << "[ERR]"; break;
                case LogLevel::Critical: std::cout << "[CRT]"; break;
                default: /* None or invalid */ break; 
            }
            std::cout << "[" << module << "] " << message << std::endl;
        }
    }

    // Variadic template log function for printf-style formatting (C++11 onwards)
    // Example: LOG_INFO_F("MyModule", "Value is %d and string is %s", 123, "hello");
    template<typename... Args>
    static void log_f(LogLevel level, const std::string& module, const char* format, Args... args) {
        Logger& instance = get();
         if (level >= instance.current_level_ && instance.current_level_ != LogLevel::None) {
            // Determine buffer size needed for formatted string
            // Using a temporary buffer. For very long strings, this might need adjustment or dynamic allocation.
            // snprintf returns the number of characters that *would have been written* if the buffer was large enough.
            int size_s = std::snprintf(nullptr, 0, format, args...);
            if (size_s <= 0) { // Error in format or no output
                log(LogLevel::Error, "Logger", "Error in log_f format string or arguments.");
                return;
            }
            auto size = static_cast<size_t>(size_s);
            std::vector<char> buf(size + 1); // +1 for null terminator
            std::snprintf(buf.data(), buf.size(), format, args...);
            log(level, module, std::string(buf.data(), buf.data() + size)); // Construct string from buffer
        }
    }


private:
    Logger() = default;
    ~Logger() = default;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    static Logger& get() {
        static Logger instance;
        return instance;
    }

    LogLevel current_level_ = LogLevel::Debug;
    std::mutex log_mutex_;
};

// Macros for convenient logging
// Simple string version
#define LOG_DEBUG(module, message)   RDT::Logger::log(RDT::LogLevel::Debug,   (module), (message))
#define LOG_INFO(module, message)    RDT::Logger::log(RDT::LogLevel::Info,    (module), (message))
#define LOG_WARN(module, message)    RDT::Logger::log(RDT::LogLevel::Warning, (module), (message))
#define LOG_ERROR(module, message)   RDT::Logger::log(RDT::LogLevel::Error,   (module), (message))
#define LOG_CRITICAL(module, message) RDT::Logger::log(RDT::LogLevel::Critical,(module), (message))

// Formatted version
#define LOG_DEBUG_F(module, ...)   RDT::Logger::log_f(RDT::LogLevel::Debug,   (module), __VA_ARGS__)
#define LOG_INFO_F(module, ...)    RDT::Logger::log_f(RDT::LogLevel::Info,    (module), __VA_ARGS__)
#define LOG_WARN_F(module, ...)    RDT::Logger::log_f(RDT::LogLevel::Warning, (module), __VA_ARGS__)
#define LOG_ERROR_F(module, ...)   RDT::Logger::log_f(RDT::LogLevel::Error,   (module), __VA_ARGS__)
#define LOG_CRITICAL_F(module, ...) RDT::Logger::log_f(RDT::LogLevel::Critical,(module), __VA_ARGS__)

} // namespace RDT
#endif // LOGGER_H