#include "../include/cmd_config.hpp"
#include "../include/command.hpp"

uint16_t CommandConfig::addrCounter = 0;
uint8_t CommandConfig::pkgAddrCounter = 0;

std::forward_list<Command*> *PackageConfig::getSetupCommands() {
    std::string gainNames[6] {"p", "i", "d", "vel", "accel", "jerk"};
    json args = {
        {"type", "gains"},
    };
    for(int i = 0; i < 6; i++) {
        std::string curGain = gainNames[i];
        if(doc[curGain].is_number()) {
            args[curGain] = doc[curGain];
        }
    }
    auto cmds = new std::forward_list<Command*>();
    cmds->push_front(new PackageCommand(*this, args));
    return cmds;
}