#include "../include/command.hpp"


std::pair<int, uint8_t*> interpRelaxCommand(uint16_t addr, uint8_t pkgAddr, bool relax) {
    int len = 7;
    auto *buf = new uint8_t[len] {
        OP_COMMAND,
        (uint8_t)(addr >> 8), 
        (uint8_t)addr,
        pkgAddr, 
        (uint8_t)(CMD_RELAX >> 8), 
        (uint8_t)CMD_RELAX,
        0x1,
        relax
    };
    return std::pair<int, uint8_t*>(len, buf);
}

std::pair<int, uint8_t*> interpSensorCommand(uint16_t addr, uint8_t pkgAddr, bool &cmdDone) {
    if(cmdDone) {
        return std::pair<int, uint8_t*>(0, NULL);
    }
    int len = 7;
    auto *buf = new uint8_t[len] {
        OP_COMMAND,
        (uint8_t)(addr >> 8), 
        (uint8_t)addr,
        pkgAddr, 
        (uint8_t)(CMD_SENSOR >> 8), 
        (uint8_t)CMD_SENSOR,
        0x0
    };
    cmdDone = true;
    return std::pair<int, uint8_t*>(len, buf);
}