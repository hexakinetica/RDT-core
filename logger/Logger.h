// Logger.h (patched)
#ifndef LOGGER_H
#define LOGGER_H
#pragma once

#include <iostream>
#include <string>
#include <string_view>
#include <mutex>
#include <vector>
#include <chrono>
#include <iomanip>
#include <cstdio>
#include <ctime>
#include <utility>   // std::forward

namespace RDT {

enum class LogLevel { Debug, Info, Warning, Error, Critical, None };

class Logger {
public:
    static void setLogLevel(LogLevel level) { get().current_level_ = level; }

    static void log(LogLevel level, const std::string& module, const std::string& message) {
        Logger& instance = get();
        if (instance.current_level_ == LogLevel::None || level < instance.current_level_) return;

        std::lock_guard<std::mutex> lock(instance.log_mutex_);

        auto now = std::chrono::system_clock::now();
        auto t   = std::chrono::system_clock::to_time_t(now);
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::tm tm_buf{};
        localtime_r(&t, &tm_buf);

        std::ostream& out = std::cout;
        out << '[' << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
            << '.' << std::setfill('0') << std::setw(3) << ms.count() << ']';

        switch (level) {
            case LogLevel::Debug:    out << "[DBG]"; break;
            case LogLevel::Info:     out << "[INF]"; break;
            case LogLevel::Warning:  out << "[WRN]"; break;
            case LogLevel::Error:    out << "[ERR]"; break;
            case LogLevel::Critical: out << "[CRT]"; break;
            case LogLevel::None:     break;
        }
        out << '[' << module << "] " << message << std::endl;
    }

    // printf-style logging with safe no-args branch
    template <typename... Args>
    static void log_f(LogLevel level, const std::string& module, const char* fmt, Args&&... args) {
        Logger& instance = get();
        if (instance.current_level_ == LogLevel::None || level < instance.current_level_) return;

        if constexpr (sizeof...(Args) == 0) {
            // No format args: treat fmt as a plain message to avoid format-security warning
            log(level, module, std::string{fmt ? fmt : ""});
        } else {
            int needed = std::snprintf(nullptr, 0, fmt, std::forward<Args>(args)...);
            if (needed <= 0) {
                log(LogLevel::Error, "Logger", "formatting error in log_f");
                return;
            }
            std::vector<char> buf(static_cast<size_t>(needed) + 1);
            std::snprintf(buf.data(), buf.size(), fmt, std::forward<Args>(args)...);
            log(level, module, std::string(buf.data(), static_cast<size_t>(needed)));
        }
    }

private:
    Logger() = default; ~Logger() = default;
    Logger(const Logger&) = delete; Logger& operator=(const Logger&) = delete;

    static Logger& get() {
        static Logger instance; return instance;
    }

    LogLevel   current_level_ = LogLevel::Debug;
    std::mutex log_mutex_;
};

// Convenience macros
#define LOG_DEBUG(module, message)     ::RDT::Logger::log(::RDT::LogLevel::Debug,    (module), (message))
#define LOG_INFO(module, message)      ::RDT::Logger::log(::RDT::LogLevel::Info,     (module), (message))
#define LOG_WARN(module, message)      ::RDT::Logger::log(::RDT::LogLevel::Warning,  (module), (message))
#define LOG_ERROR(module, message)     ::RDT::Logger::log(::RDT::LogLevel::Error,    (module), (message))
#define LOG_CRITICAL(module, message)  ::RDT::Logger::log(::RDT::LogLevel::Critical, (module), (message))

#define LOG_DEBUG_F(module, ...)       ::RDT::Logger::log_f(::RDT::LogLevel::Debug,    (module), __VA_ARGS__)
#define LOG_INFO_F(module, ...)        ::RDT::Logger::log_f(::RDT::LogLevel::Info,     (module), __VA_ARGS__)
#define LOG_WARN_F(module, ...)        ::RDT::Logger::log_f(::RDT::LogLevel::Warning,  (module), __VA_ARGS__)
#define LOG_ERROR_F(module, ...)       ::RDT::Logger::log_f(::RDT::LogLevel::Error,    (module), __VA_ARGS__)
#define LOG_CRITICAL_F(module, ...)    ::RDT::Logger::log_f(::RDT::LogLevel::Critical, (module), __VA_ARGS__)

} // namespace RDT
#endif // LOGGER_H
