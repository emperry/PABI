#include <nlohmann/json.hpp>
#include "../include/command.hpp"
using json = nlohmann::json;
#include <signal.h>

#define JSON_NAN_OR_FLOAT(J,KEY) \
    J[KEY].is_number() ? (float)(J[KEY]) : NAN

/* Package command */
PackageCommand::PackageCommand(PackageConfig &config, json &args)
    : Command(config.id), config(config) {
    std::string type = args["type"];
    cmdType = type;
    if(type.compare("setpoint") == 0) {
        std::string sType = args["setpoint_type"];
        if(sType.compare("pos") == 0) {
            setpointType = CMD_POSITION;
        } else {
            throw std::invalid_argument("invalid setpoint type argument");
        }
        auto d = args["data"];
        for(auto elem : d) {
            float nElem;
            // TODO: cubic spline interpolation
            auto t = std::chrono::milliseconds(0); 
            if(elem.is_array()) {
                // for when theres a time delay present as well
                nElem = config.actuator.clampIfOutside(elem[0]);
                t += std::chrono::milliseconds((uint)elem[1]);
            } else {
                nElem = config.actuator.clampIfOutside(elem);
            }
            float past = config.actuator.getCurPos();
            if(setpoints.size() > 0) {
                past = setpoints.front().first;
            }
            t += config.actuator.calcTime(past, nElem);
            //config.actuator.curPos = nElem;
            setpoints.push_front(std::pair<float,std::chrono::milliseconds>(nElem, t));
            setLen++;
        }
    } else if(type.compare("gains") == 0) {
        for(int i = 0; i < 6; i++) {
            float cur = JSON_NAN_OR_FLOAT(args,gainNames[i]);
            if(!std::isnan(cur)) {
                gainsAndCmd.push_back(
                    std::pair<float,int>(cur,gainToFloat(gainNames[i])));
                setLen++;
            }
        }
        if(setLen == 0) {
            throw std::invalid_argument("no gains specified");
        }
    } else {
        throw std::invalid_argument("invalid type argument");
    }
    timer = std::chrono::steady_clock::now();
}

int PackageCommand::gainToFloat(std::string name) {
    if(name.compare("p") == 0) {
        return CMD_P_GAIN;
    } else if(name.compare("i") == 0) {
        return CMD_I_GAIN;
    } else if(name.compare("d") == 0) {
        return CMD_D_GAIN;
    } else if(name.compare("vel") == 0) {
        return CMD_VEL_GAIN;
    } else if(name.compare("accel") == 0) {
        return CMD_ACC_GAIN;
    } else if(name.compare("jerk") == 0) {
        return CMD_JERK_GAIN;
    }
    throw std::invalid_argument("unknown gain specified");
}

std::pair<int, uint8_t*> PackageCommand::interpret() {
    if(cmdRun) {
        return std::pair<int, uint8_t*>(0, NULL);
    }
    uint16_t address = config.actuator.getAddr();
    uint8_t pkgAddr = config.getPkgAddr();
    uint8_t size = 15;
    int32_t curData = 0;
    int32_t curV = 0;
    if(cmdType.compare("setpoint") == 0) {
        auto cur = setpoints.back();
        memcpy(&curData,&cur.first,4);
        float curVFloat = std::abs(config.actuator.velocity);
        memcpy(&curV, &curVFloat, 4);
        size = 15;
    } else if(cmdType.compare("gains") == 0) {
        auto cur = gainsAndCmd.back();
        memcpy(&curData,&cur.first,4);
        setpointType = cur.second;
        size = 11;
    }
    auto out = new uint8_t[size];
    out[0] = OP_COMMAND;
    out[1] = address >> 8;
    out[2] = address;
    out[3] = pkgAddr;
    out[4] = setpointType >> 8;
    out[5] = setpointType;
    if(cmdType.compare("setpoint") == 0) {
        out[6] = 0x08;
        out[7] = curData >> 24;
        out[8] = curData >> 16;
        out[9] = curData >> 8;
        out[10] = curData;
        out[11] = curV >> 24;
        out[12] = curV >> 16;
        out[13] = curV >> 8;
        out[14] = curV;
    } else {
        out[6] = 0x04;
        out[7] = curData >> 24;
        out[8] = curData >> 16;
        out[9] = curData >> 8;
        out[10] = curData;
    }
    cmdRun = true;
    return std::pair<int, uint8_t*>(size, out);
}

bool PackageCommand::step(std::chrono::microseconds loopTime) {
    bool isSetpoint = cmdType.compare("setpoint") == 0;
    if(isSetpoint) {
        // interpolate joint position so overriding commands don't wait too long
        auto seconds = ((float)loopTime.count())/1e6;
        auto dx = config.actuator.velocity * seconds;
        float curPos = config.actuator.getCurPos();
        dx *= setpoints.back().first > curPos ? 1 : -1;
        config.actuator.setCurPos(curPos + dx);
        //std::cout << config.actuator.curPos << std::endl;
    }
    if(setLen - 1 == setPos) {
        // wait on the last time
        if(isSetpoint &&
           (std::chrono::steady_clock::now() - timer) < setpoints.back().second) {
            config.actuator.setCurPos(setpoints.back().first);
            return true;
        }
        return false;
    }
    if(setPos == -1) {
        setPos = 0;
        return true;
    }
    if(isSetpoint && 
       (std::chrono::steady_clock::now() - timer) > setpoints.back().second) {
        // keep the last one
        if(setLen - 1 > setPos) {
            setpoints.pop_back();
        }
        //config.actuator.curPos = setpoints.back().first;
        timer = std::chrono::steady_clock::now();
        setPos++;
        cmdRun = false;
    } else if(cmdType.compare("gains") == 0) {
        if(setPos != -1) {
            gainsAndCmd.pop_back();
        }
        setPos++;
        cmdRun = false;
    }
    return true;
}

/* Sensor feedback command */
SensorCommand::SensorCommand(FeedbackConfig &config, json &args): 
    Command(config.id), config(config), 
    sampler(CommandSampler(args)) {}

std::pair<int, uint8_t*> SensorCommand::interpret() {
    return interpSensorCommand(config.getAddr(), config.getPkgAddr(), sampler.cmdDone);
}

void SensorCommand::aggregateResponses(json &acc, json &res, int resNumber) {
    sampler.aggregateResponses(acc, res, resNumber, 
        [this](float in) { return config.scaleAndZero(in); });
}

bool SensorCommand::step(std::chrono::microseconds loopTime) {
    return sampler.step();
}

int SensorCommand::getTotalSteps() {
    return sampler.getTotalSteps();
}

// Sampler helper class
CommandSampler::CommandSampler(json &args): steps(0) {
    if(args.is_object() && args["sample_rate"].is_number() && args["sample_interval"].is_number()) {
        sampling = true;
        sampleRate = std::chrono::milliseconds(args["sample_rate"]);
        sampleInterval = std::chrono::milliseconds(args["sample_interval"]);
        timer = std::chrono::steady_clock::now();
        if(sampleInterval <= std::chrono::milliseconds(0) || sampleRate <= std::chrono::milliseconds(0)) {
            throw std::logic_error("sample interval or sample rate is an invalid value");
        }
        totalSteps = (int)(sampleInterval / sampleRate);
    }
}

void CommandSampler::aggregateResponses(json &acc, json &res, int resNumber, 
        std::function<float(float)> transResponse) {
    if(!acc["data"].is_array()) {
        acc["data"] = {};
    }
    if(res["response"].is_number()) {
        float curRes = res["response"];
        acc["data"].push_back(transResponse(curRes));
    }
}

bool CommandSampler::step() {
    if(!sampling) {
        return !cmdDone;
    }
    if(steps == totalSteps) {
        return false;
    }
    if(std::chrono::steady_clock::now() - timer > sampleRate) {
        cmdDone = false;
        std::cout << "STEPS " << steps+1 << std::endl;
        steps++;
        timer = std::chrono::steady_clock::now();
    }
    return true;
}

int CommandSampler::getTotalSteps() {
    if(sampling) {
        return totalSteps;
    } else {
        return 1;
    }
}