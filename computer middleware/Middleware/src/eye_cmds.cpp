#include "../include/command.hpp"

#define EYE_FOCUS_CALC(XY,Z,F) \
    ((XY * F) / (F + Z))

/*double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}*/

/* Eye command */
EyeCommand::EyeCommand(EyeConfig &config, json &args)
    : Command(config.id), config(config) {
    type = args["type"];
    if(type.compare("blink") == 0) {
        if(args["frames"].is_null()) {
            frames = 5;
        } else if(!args["frames"].is_number_unsigned()) {
            throw std::invalid_argument("number of frames for a blink should be an unsigned int");
        } else {
            frames = args["frames"];
        }
    } else if(type.compare("move") == 0) {
        // Retrieve x, y, z from the arguments
        double x = args["x"];
        x *= -1; // So PABI left == observer left
        double y = args["y"];
        double z = 1;
        if(args["z"].is_number()) {
            z = args["z"];
        }
        auto vL = new double[4] {x, y, z, 1};
        auto vR = new double[4] {x, y, z, 1};
        // Camera -> Screen transforms
        config.leftCT.multVec(vL);
        config.rightCT.multVec(vR);
        // Focal point calculations
        double f = 5;
        vL[0] = EYE_FOCUS_CALC(vL[0],vL[2],f);
        vR[0] = EYE_FOCUS_CALC(vR[0],vR[2],f);
        vL[1] = EYE_FOCUS_CALC(vL[1],vL[2],f);
        vR[1] = EYE_FOCUS_CALC(vR[1],vR[2],f);
        // Screen coords -> Pixel coords transform
        config.leftPT.multVec(vL);
        config.rightPT.multVec(vR);
        // Clamp to 0, 160
        /*vL[0] = clip(vL[0], 0, 160);
        vR[0] = clip(vR[0], 0, 160);
        vL[1] = clip(vL[1], 0, 160);
        vR[1] = clip(vR[1], 0, 160);*/
        std::cout << "L: " << vL[0] << " " << vL[1] << " R: " <<
        vR[0] << " " << vR[1] << std::endl;
        x1 = vL[0];
        y1 = vL[1];
        x2 = vR[0];
        y2 = vR[1];
        delete[] vL;
        delete[] vR;
    } else {
        throw std::invalid_argument("invalid type for eye command");
    }
}

std::pair<int, uint8_t*> EyeCommand::interpret() {
    int size;
    if(type.compare("move") == 0) {
        size = 11;
    } else if(type.compare("blink") == 0) {
        size = 8;
    } else {
        throw std::invalid_argument("eye command type not implemented");
    }
    uint16_t addr = config.getAddr();
    uint8_t pkgAddr = config.getPkgAddr();
    auto *cmd = new uint8_t[size];
    cmd[0] = OP_COMMAND;
    cmd[1] = addr >> 8;
    cmd[2] = addr;
    cmd[3] = pkgAddr;
    if(type.compare("move") == 0) {
        cmd[4] = (uint8_t)(CMD_NEW_EYE_POS >> 8);
        cmd[5] = (uint8_t)CMD_NEW_EYE_POS;
        cmd[6] = 0x4;
        cmd[7] = x1;
        cmd[8] = y1;
        cmd[9] = x2;
        cmd[10] = y2;
    } else if(type.compare("blink") == 0) {
        cmd[4] = (uint8_t)(CMD_EYE_BLINK >> 8);
        cmd[5] = (uint8_t)(CMD_EYE_BLINK);
        cmd[6] = 0x01;
        cmd[7] = frames;
    }
    return std::pair<int, uint8_t*>(size, cmd);
}