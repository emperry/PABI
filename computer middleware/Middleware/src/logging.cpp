#include "../include/logging.hpp"

Logger::Logger(std::string path): stream() {
    stream.open(path, std::ofstream::out);
    stream << "I" << std::endl;
    begin = std::chrono::steady_clock::now();
}

void Logger::writeByteLine(char prefix, uint8_t *bytes, int len) {
    writeLineBegin(prefix);
    for(int i = 0; i < len; i++) {
        stream << std::hex << (int)(bytes[i]);
    }
    stream << std::endl;
}

void Logger::writeStrLine(char prefix, std::string line) {
    writeLineBegin(prefix);
    stream << line << std::endl;
}

void Logger::writeLineBegin(char prefix) {
    auto now = std::chrono::steady_clock::now();
    auto t = std::chrono::duration_cast<std::chrono::microseconds>(now - begin);
    begin = now;
    stream << std::dec << t.count() << " ";
    stream << prefix << " ";
}

Logger::~Logger() {
    stream.close();
}