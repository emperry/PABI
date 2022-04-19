#include <string>
#include <cstdint>
#include <forward_list>
#include <iostream>
#include <cstring>
#include <set>
#include <nlohmann/json.hpp>
#include <chrono>
#include "comms.hpp"
#include "mat.hpp"
using json = nlohmann::json;

#ifndef COMMAND_CONFIG_H
#define COMMAND_CONFIG_H

class Command;

class CommandConfig {
    public:
        CommandConfig(std::string id): id(id) {
            this->addr = addrCounter++;
        }
        // Control over weather to increment the id or not
        // non incrementing the id should only be used for
        // devices that can never be called by themselves
        CommandConfig(std::string id, bool incId): id(id) {
            if(incId) {
                this->addr = addrCounter++;
            } else {
                this->addr = -1;
            }
        }
        enum Type {
            Actuator,
            Feedback,
            Eye,
            Package,
            Arm,
            SavedCommand
        };
        virtual enum CommandConfig::Type getCmdType() = 0;
        // Returns the initialization command for this device
        // if the return length is -1 there is no initialization command
        // otherwise it returns the new length of out
        // and a pointer to the command array
        virtual std::pair<int, uint8_t*> getInitCmd() = 0;
        // Returns command objects to be run before anything happens
        virtual std::forward_list<Command*> *getSetupCommands() {
            return NULL;
        }
        virtual ~CommandConfig() {};
        // Returns a unique number that identifies the device
        uint16_t getAddr() {
            return addr;
        }
        uint8_t getPkgAddr() {
            return pkgAddr;
        }
        // Retrieves the full address [<empty, addr1, addr2, pkg] of this command
        uint32_t getFullAddr() {
            uint32_t fullAddr = 0;
            fullAddr |= getAddr() << 8;
            fullAddr |= getPkgAddr();
            return fullAddr;
        }
        // Put this device in a new package
        int makeNewPkg() {
            pkgAddrCounter++;
            this->pkgAddr = pkgAddrCounter;
            return pkgAddrCounter;
        }
        // Sets the package id
        void setPkgAddr(uint8_t pkgAddr) {
            this->pkgAddr = pkgAddr;
        }
        // The unique string that identifies
        // the device in the database
        std::string id;
        // scale factor to apply before sending each command
        float scaleFactor;
        // zeroed position, this number is subtracted 
        // from every sent and returned value
        float zeroedPosition;
    private:
        uint16_t addr;
        uint8_t pkgAddr = 0;
        static uint16_t addrCounter;
        static uint8_t pkgAddrCounter;
};

class ActuatorConfig : public CommandConfig {
    public:
        ActuatorConfig(std::string id, float minPos, float maxPos): 
            CommandConfig(id), minPos(minPos), maxPos(maxPos) {
                if(minPos >= maxPos) {
                    throw std::logic_error("minimum position higher than max on " + id);
                }
                // initial position is neutral
                curPos = (maxPos - minPos)/2;
            };
        enum CommandConfig::Type getCmdType() {
            return CommandConfig::Type::Actuator; 
        }
        virtual std::pair<int, uint8_t*> getInitCmd() = 0;
        float minPos, maxPos;
        float velocity = 0;
        void setCurPos(float newPos) {
            curPos = newPos;
        }
        float getCurPos() {
            return curPos;
        }
        // parses input to this actuator
        // applies the scale factor, zeroed position and
        // throws an exception if it's outside bounds
        float parseValue(float in) {
            if(in > maxPos || in < minPos) {
                throw std::runtime_error("actuator position for " + 
                    id + " outside range");
            }
            in = scaleAndZero(in);
            return in;
        }
        // returns true if outside range, false if it isn't
        bool isOutsideRange(float in) {
            return in > maxPos || in < minPos;
        }
        // clamps to outside range if outside, does scale/zeroing
        float clampIfOutside(float in) {
            if(in > maxPos) {
                return scaleAndZero(maxPos);
            } else if(in < minPos) {
                return scaleAndZero(minPos);
            }
            in = scaleAndZero(in);
            return in;
        }
        // calculate time to get to position from other position
        std::chrono::milliseconds calcTime(float from, float to) {
            return std::min(std::chrono::milliseconds((uint)((std::abs(to-from)/velocity)*1000)),
                std::chrono::milliseconds(2500)); // TODO: factor out min time?
            //return std::chrono::milliseconds((uint)((std::abs(to-from)/velocity)*1000));
        }
        std::chrono::milliseconds calcToNext(float to) {
            return calcTime(curPos, to);
        }
        // calculates velocity needed to travel between from and two at the given speed
        float calcVel(float from, float to, std::chrono::milliseconds t) {
            return (to-from)/((float)t.count()/1000.0);
        }
        float calcVelToNext(float to, std::chrono::milliseconds t) {
            return calcVel(curPos, to, t);
        }
        inline float reverseScaleAndZero(float in) {
            in -= zeroedPosition;
            in *= scaleFactor;
            return in;
        }
        ~ActuatorConfig() {};
    private:
        float curPos = 0;
        inline float scaleAndZero(float in) {
            in /= scaleFactor;
            in += zeroedPosition;
            return in;
        }
};

class BrushedMotorConfig : public ActuatorConfig {
    public:
        BrushedMotorConfig(std::string id, float minPos, float maxPos,
        uint8_t pwmPin)
        : ActuatorConfig(id, minPos, maxPos), pwmPin(pwmPin) {};
        uint8_t pwmPin;
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 7;
            uint16_t addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_BRUSHED_DC, 
                (uint8_t)(addr >> 8), 
                (uint8_t)addr, getPkgAddr(), 0x1, 
                pwmPin};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~BrushedMotorConfig() {};
};

class PWMServoConfig : public ActuatorConfig {
    public:
        PWMServoConfig(std::string id, float minPos, float maxPos,
        uint8_t pwmPin, float maxRot)
        : ActuatorConfig(id, minPos, maxPos), pwmPin(pwmPin),
        maxRot(maxRot) {};
        uint8_t pwmPin;
        float maxRot;
        std::pair<int, uint8_t*> getInitCmd() {
            uint32_t rot;
            memcpy(&rot, &maxRot, 4);
            int len = 11;
            uint16_t addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_PWM_SERVO, 
            (uint8_t)(addr >> 8), 
            (uint8_t)addr, getPkgAddr(), 0x5, pwmPin,
            (uint8_t)(rot >> 24), (uint8_t)(rot >> 16), 
            (uint8_t)(rot >> 8), (uint8_t)(rot)};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~PWMServoConfig() {};
};

class BrushlessMotorConfig : public ActuatorConfig {
    public:
        BrushlessMotorConfig(std::string id, float minPos, 
        float maxPos, uint8_t pin)
        : ActuatorConfig(id, minPos, maxPos), pin(pin) {};
        uint8_t pin;
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 7;
            uint16_t addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_BRUSHLESS, 
            (uint8_t)(addr >> 8), 
            (uint8_t)addr, getPkgAddr(), 0x1, pin};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~BrushlessMotorConfig() {};
};

class XYZSmartServoConfig : public ActuatorConfig {
    public:
        XYZSmartServoConfig(std::string id, float minPos, float maxPos, uint8_t uartAddr)
        : ActuatorConfig(id, minPos, maxPos), uartAddr(uartAddr) {};
        uint8_t uartAddr;
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 7;
            uint16_t addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_XYZ_SMART_SERVO, 
                (uint8_t)(addr >> 8), 
                (uint8_t)addr, getPkgAddr(), 0x1, uartAddr};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~XYZSmartServoConfig() {};
};

class DynamixelServoConfig : public ActuatorConfig {
    public:
        DynamixelServoConfig(std::string id, float minPos, float maxPos, uint8_t uartAddr)
        : ActuatorConfig(id, minPos, maxPos), uartAddr(uartAddr) {};
        uint8_t uartAddr;
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 7;
            uint16_t addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_DYNAMIXEL_SERVO, 
                (uint8_t)(addr >> 8),
                (uint8_t)addr, getPkgAddr(), 0x1, uartAddr};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~DynamixelServoConfig() {};
}

class DynamixelSensorConfig : public FeedbackConfig {
    public:
    DynamixelSensorConfig(std::string id, uint8_t uartAddr):
            FeedbackConfig(id), uartAddr(uartAddr) {};
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 7;
            auto addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_DYNAMIXEL_SENSOR,
                (uint8_t)(addr >> 8), (uint8_t)addr, getPkgAddr(),
                0x1, uartAddr};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~DynamixelSensorConfig() {};
    private:
        uint8_t uartAddr;
};

class StepperMotorConfig : public ActuatorConfig {
    public:
        StepperMotorConfig(std::string id, uint8_t stepPin,
        float minPos, float maxPos, uint8_t dirPin, uint8_t enPin)
        : ActuatorConfig(id, minPos, maxPos), stepPin(stepPin), dirPin(dirPin),
        enPin(enPin) {};
        uint8_t stepPin, dirPin, enPin;
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 9;
            uint16_t addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_STEPPER, 
                (uint8_t)(addr >> 8), 
                (uint8_t)addr, getPkgAddr(), 
                0x3, stepPin, dirPin, enPin};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~StepperMotorConfig() {};
};

class BinaryOutputConfig : public ActuatorConfig {
    public:
        BinaryOutputConfig(std::string id, uint8_t pin):
            ActuatorConfig(id, 0, 1), pin(pin) {};
        uint8_t pin;
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 7;
            uint16_t addr = getAddr();
            auto cmd = new uint8_t[len] { OP_MODCONF, CMD_BIN_OUT,
            (uint8_t)(addr >> 8), 
            (uint8_t)addr, getPkgAddr(),
            0x1, pin};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~BinaryOutputConfig() {};
};

class FeedbackConfig : public CommandConfig {
    public:
        FeedbackConfig(std::string id): CommandConfig(id) {};
        enum CommandConfig::Type getCmdType() {
            return CommandConfig::Type::Feedback; 
        }
        virtual std::pair<int, uint8_t*> getInitCmd() = 0;
        inline float scaleAndZero(float in) {
            in -= zeroedPosition;
            in *= scaleFactor;
            return in;
        }
        ~FeedbackConfig() {};
};

class SPIEncoderConfig : public FeedbackConfig {
    public:
        SPIEncoderConfig(std::string id, uint8_t chainSize, uint8_t chainAddr):
            FeedbackConfig(id), chainSize(chainSize), chainAddr(chainAddr) {};
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 8;
            auto addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_SPIENC, 
                (uint8_t)(addr >> 8), (uint8_t)addr, getPkgAddr(),
                0x2, chainSize, chainAddr};
            return std::pair<int, uint8_t*>(len, cmd);
        }
    private:
        uint8_t chainSize, chainAddr;
};

class PotentiometerConfig : public FeedbackConfig {
    public:
        PotentiometerConfig(std::string id, uint8_t pin):
            FeedbackConfig(id), pin(pin) {};
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 7;
            auto addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_POTENTIOMETER, 
                (uint8_t)(addr >> 8), (uint8_t)addr, getPkgAddr(),
                0x1, pin};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~PotentiometerConfig() {};
    private:
        uint8_t pin;
};

class EncoderConfig : public FeedbackConfig {
    public:
        EncoderConfig(std::string id, uint8_t pin, 
            uint16_t ticksPerRev):
            FeedbackConfig(id), pin(pin), 
            ticksPerRev(ticksPerRev) {};
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 8;
            auto addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_QUAD_ENC, 
                (uint8_t)(addr >> 8), (uint8_t)addr, getPkgAddr(),
                0x3, pin, (uint8_t)(ticksPerRev >> 8), (uint8_t)(ticksPerRev)};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~EncoderConfig() {};
    private:
        uint8_t pin;
        uint8_t ticksPerRev;
};

class BinaryInputConfig : public FeedbackConfig {
    public:
        BinaryInputConfig(std::string id, 
            uint8_t pin, bool normallyOpen):
            FeedbackConfig(id), pin(pin), normallyOpen(normallyOpen) {};
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 8;
            auto addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_LIMIT_SWITCH, 
                (uint8_t)(addr >> 8), (uint8_t)addr, getPkgAddr(),
                0x2, pin, (uint8_t)normallyOpen};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~BinaryInputConfig() {};
    private:
        uint8_t pin;
        bool normallyOpen;
};

class XYZSmartSensorConfig : public FeedbackConfig {
    public:
        XYZSmartSensorConfig(std::string id, uint8_t uartAddr):
            FeedbackConfig(id), uartAddr(uartAddr) {};
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 7;
            auto addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_XYZ_SMART_SENSOR, 
                (uint8_t)(addr >> 8), (uint8_t)addr, getPkgAddr(),
                0x1, uartAddr};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        ~XYZSmartSensorConfig() {};
    private:
        uint8_t uartAddr;
};

class EyeConfig : public CommandConfig {
    public:
        EyeConfig(std::string id, std::string leftPixelTrans,
            std::string rightPixelTrans, std::string leftCameraTrans,
            std::string rightCameraTrans,
            uint8_t **backgroundImg, uint8_t **innerImg, int imgW, int imgH): 
                CommandConfig(id),
                backgroundImg(backgroundImg), innerImg(innerImg),
                imgW(imgW), imgH(imgH) {
                    makeNewPkg();
                    // Generate transformation matrix
                    leftPT = transformStringToMat(leftPixelTrans);
                    rightPT = transformStringToMat(rightPixelTrans);
                    leftCT = transformStringToMat(leftCameraTrans);
                    rightCT = transformStringToMat(rightCameraTrans);
                };
        TransMatrix leftPT, rightPT, leftCT, rightCT;
        uint8_t **backgroundImg;
        uint8_t **innerImg;
        int imgW;
        int imgH;
        std::pair<int, uint8_t*> getInitCmd() {
            int len = 6;
            auto addr = getAddr();
            auto *cmd = new uint8_t[len] {OP_MODCONF, CMD_LCD_DISPLAY, 
                (uint8_t)(addr >> 8), (uint8_t)addr, getPkgAddr(), 0x0};
            return std::pair<int, uint8_t*>(len, cmd);
        }
        enum CommandConfig::Type getCmdType() { 
            return CommandConfig::Type::Eye; 
        }
        ~EyeConfig() {
            // TODO: delete images
        };
};

class PackageConfig : public CommandConfig {
    public:
        PackageConfig(std::string id,
            ActuatorConfig &actuator, FeedbackConfig *feedback, json &doc):
                CommandConfig(id, false), actuator(actuator), feedback(feedback), 
                doc(doc) {
                    auto pkgAddr = makeNewPkg();
                    actuator.setPkgAddr(pkgAddr);
                    feedback->setPkgAddr(pkgAddr);
                };
        PackageConfig(std::string id, ActuatorConfig &actuator, json &doc):
            CommandConfig(id, false), actuator(actuator), doc(doc) {
                auto pkgAddr = makeNewPkg();
                actuator.setPkgAddr(pkgAddr);
            };
        ActuatorConfig &actuator;
        FeedbackConfig *feedback = NULL;
        std::pair<int, uint8_t*> getInitCmd() {
            return std::pair<int, uint8_t*>(-1, NULL);
        }
        enum CommandConfig::Type getCmdType() { 
            return CommandConfig::Type::Package; 
        }
        std::forward_list<Command*> *getSetupCommands();
        void setMatrixStr(std::string matString) {
            matChain = symStringToMat(matString);
        }
        std::vector<SymTransMatrix*> matChain;
        ~PackageConfig() {
            for(auto m : matChain) {
                delete m;
            }
        };
    private:
        json doc;
};

class ArmConfig : public CommandConfig {
    public:
        ArmConfig(std::string id, std::vector<PackageConfig*> jointPackages):
            CommandConfig(id, false), jointPackages(jointPackages) {
            // Create SymTransMatrix chain and store
            for(auto pkg : jointPackages) {
                if(!pkg->feedback) {
                    throw std::invalid_argument("every joint in "+id+
                        " must have a feedback sensor");
                }
                if(pkg->matChain.size() > 0) {
                    auto chain = pkg->matChain;
                    for(auto mat : chain) {
                    //for(auto i = chain.rbegin(); i != chain.rend(); i++) {
                        //auto mat = *i;
                        matrixChain.push_back(mat);
                        auto curVar = mat->getSymName();
                        if(curVar.compare("") != 0) {
                            matVars.insert(curVar);
                        }
                    }
                }
            }
        };
        std::vector<PackageConfig*> jointPackages;
        std::pair<int, uint8_t*> getInitCmd() {
            return std::pair<int, uint8_t*>(-1, NULL);
        }
        enum CommandConfig::Type getCmdType() { 
            return CommandConfig::Type::Arm; 
        }
        std::vector<SymTransMatrix*> matrixChain;
        std::set<std::string> matVars;
        ~ArmConfig() {};
};

class SavedCommandConfig : public CommandConfig {
    public:
        SavedCommandConfig(std::string id, json doc):
            CommandConfig(id, false), saved(doc) {};
        std::pair<int, uint8_t*> getInitCmd() {
            return std::pair<int, uint8_t*>(-1, NULL); 
        }
        enum CommandConfig::Type getCmdType() { 
            return CommandConfig::Type::SavedCommand; 
        }
        // Returns the command ids and arguments for the saved command
        std::forward_list<std::pair<std::string,json&>> *getCmds() {
            auto cmds = new std::forward_list<std::pair<std::string,json&>>();
            for(auto it = saved["move"].begin(); it != saved["move"].end(); ++it) {
                cmds->push_front(std::pair<std::string,json&>(it.key(),it.value()));
            }
            return cmds;
        }
        ~SavedCommandConfig() {};
    private:
        json saved;
};

#endif
