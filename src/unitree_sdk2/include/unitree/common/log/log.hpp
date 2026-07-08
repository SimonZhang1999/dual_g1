#ifndef __UT_LOG_LOG_HPP__
#define __UT_LOG_LOG_HPP__

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

namespace unitree
{
namespace common
{

class Logger
{
public:
    explicit Logger(std::string name = "unitree") : name_(std::move(name)) {}

    const std::string& Name() const { return name_; }

private:
    std::string name_;
};

inline Logger* GetLogger(const std::string& name)
{
    return new Logger(name);
}

inline void ReleaseLogger(Logger* logger)
{
    delete logger;
}

template <typename... Args>
inline void LogImpl(const char* level, Logger* logger, Args&&... args)
{
    static std::mutex log_mutex;

    std::ostringstream ss;
    (ss << ... << std::forward<Args>(args));

    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::lock_guard<std::mutex> guard(log_mutex);
    std::cerr << "[" << std::put_time(std::localtime(&t), "%F %T") << "]"
              << "[" << level << "]";
    if (logger)
    {
        std::cerr << "[" << logger->Name() << "]";
    }
    std::cerr << " " << ss.str() << std::endl;
}

}  // namespace common
}  // namespace unitree

#define LOG_INFO(logger, ...) ::unitree::common::LogImpl("INFO", (logger), __VA_ARGS__)
#define LOG_WARNING(logger, ...) ::unitree::common::LogImpl("WARN", (logger), __VA_ARGS__)
#define LOG_ERROR(logger, ...) ::unitree::common::LogImpl("ERROR", (logger), __VA_ARGS__)

#endif  // __UT_LOG_LOG_HPP__
