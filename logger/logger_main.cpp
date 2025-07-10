// Logger_main.cpp
#include "Logger.h"
#include <string>
#include <vector>

int main() {
    std::cout << "--- Logger Test ---" << std::endl;

    RDT::Logger::setLogLevel(RDT::LogLevel::Debug);
    LOG_DEBUG("TestMod", "This is a debug message.");
    LOG_INFO("TestMod", "This is an info message.");
    LOG_WARN("TestMod", "This is a warning message.");
    LOG_ERROR("TestMod", "This is an error message.");
    LOG_CRITICAL("TestMod", "This is a critical message!");

    LOG_INFO_F("Formatter", "Hello, %s! Value: %d, Float: %.2f", "World", 123, 3.14159);
    
    std::string test_str = "dynamic string";
    int test_int = 42;
    LOG_DEBUG_F("Formatter", "String: %s, Int: %d", test_str.c_str(), test_int);


    RDT::Logger::setLogLevel(RDT::LogLevel::Warning);
    LOG_INFO("TestMod", "This info message should NOT appear.");
    LOG_DEBUG("TestMod", "This debug message should NOT appear.");
    LOG_WARN("TestMod", "This warning message SHOULD appear.");

    RDT::Logger::setLogLevel(RDT::LogLevel::None);
    LOG_CRITICAL("TestMod", "This critical message should NOT appear (logging disabled).");


    std::cout << "--- Logger Test Complete ---" << std::endl;
    return 0;
}