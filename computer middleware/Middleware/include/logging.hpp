#include <string>
#include <cstdint>
#include <fstream>
#include <chrono>

#ifndef LOGGING_H
#define LOGGING_H

// allows async logging to a file 
class Logger {
    public:
        Logger(std::string path);
        void writeByteLine(char prefix, uint8_t *bytes, int len);
        void writeStrLine(char prefix, std::string line);
        ~Logger();
    private:
        void writeLineBegin(char prefix);
        std::ofstream stream;
        std::chrono::steady_clock::time_point begin;
};

#endif