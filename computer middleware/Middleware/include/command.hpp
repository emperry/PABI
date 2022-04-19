#include <cstdint>
#include "cmd_config.hpp"
#include "response.hpp"
#include "mat.hpp"
#include <math.h>
#include <random>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#ifndef COMMAND_H
#define COMMAND_H

std::pair<int, uint8_t*> interpSensorCommand(uint16_t addr, uint8_t pkgAddr, bool &cmdDone);
std::pair<int, uint8_t*> interpRelaxCommand(uint16_t addr, uint8_t pkgAddr, bool relax);

class Command {
    public:
        Command(std::string id): id(id), res(NULL) {};
        // Outputs the id associated with the underlying
        // device.
        std::string getId() {
            return this->id;
        }
        // Outputs ids associated with all devices controlled by this command
        virtual std::vector<std::string> getAssociatedIds() {
            auto v = std::vector<std::string>();
            v.push_back(this->id);
            return v;
        }
        // Outputs full addresses [<empty, addr1, addr2, pkg] to all devices
        // controlled by this command
        virtual std::vector<uint32_t> getAssociatedFullAddrs() = 0;
        // adds a callback with the argument as this command object
        // and a message
        void addCallback(Response &res) {
            this->res = &res;
        }
        // overridable function to aggregate responses together
        virtual void aggregateResponses(json &acc, json &res, int resNumber) {
            if(!res["status"].is_null()) {
                acc["status"] = res["status"];
            }
            if(!res["response"].is_null()) {
                acc["response"] = res["response"];
            }
        }
        // if finished returns true and triggers the callback
        // otherwise returns false
        bool finish(json &msg, bool fail) {
            // Aggregate intermediate responses
            aggregateResponses(resData, msg, responses);
            responses++; // new response recieved
            #ifdef PRINT_DEBUG
            std::cout << "Finish " << id << " cnt " << responses << std::endl;
            #endif
            if(res != NULL && (fail || (responses == getTotalSteps() && stepsOver))) {
                auto last = res->commandFinished(this, fail ? msg : resData);
                if(last) {
                    delete res;
                    res = NULL;
                }
                return true;
            }
            return false;
        }
        // finishes if already ended
        bool finishIfEnded() {
            if(res != NULL && responses == getTotalSteps() && stepsOver) {
                auto last = res->commandFinished(this, resData);
                if(last) {
                    delete res;
                    res = NULL;
                }
                return true;
            }
            return false;
        }
        // Outputs the command string to pass along to
        // the microcontroller. Uses data from the
        // constructor to make this.
        virtual std::pair<int, uint8_t*> interpret() = 0;
        // Increments to the next step in the command
        // returns true if the current step is new returns false
        // if there are no steps left
        virtual bool step(std::chrono::microseconds loopTime) {
            // default implementation is single step
            bool temp = stepsLeft;
            stepsLeft = false;
            return temp;
        }
        // Returns the total number of steps this command has
        virtual int getTotalSteps() { return 1; }
        virtual std::string getVerb() = 0;
        // Start/resets the timeout timer
        void resetTimer() {
            t = std::chrono::steady_clock::now();
        }
        bool isTimedOut() {
            return (std::chrono::steady_clock::now() - t) > std::chrono::seconds(3); 
        }
        // Changes the priority of the command from its default value '0'
        // lower priority is more important
        void setPriority(uint8_t priority) {
            this->priority = priority;
        }
        uint8_t getPriority() {
            return priority;
        }
        // sets the value of the last step, this should be updated in the event loop
        void setLastStep(bool lastStep) {
            stepsOver = !lastStep;
        }
        virtual ~Command() {};
    private:
        bool stepsLeft = true;
        bool stepsOver = false;
        int responses = 0;
        std::string id;
        std::chrono::steady_clock::time_point t;
        Response *res;
        json resData;
        uint8_t priority = 0;
};

class PackageCommand : public Command {
    public:
        PackageCommand(PackageConfig &config, json &args);
        std::pair<int, uint8_t*> interpret();
        bool step(std::chrono::microseconds loopTime);
        int getTotalSteps() { return setLen; }
        std::string getVerb() {
            return "move";
        }
        std::vector<uint32_t> getAssociatedFullAddrs() {
            auto v = std::vector<uint32_t>();
            // just use the actuator, feedback commands can still happen
            // in the background
            v.push_back(config.actuator.getFullAddr());
            return v;
        }
    private:
        PackageConfig &config;
        int gainToFloat(std::string name);
        std::string cmdType;
        // setpoint, delay pair
        std::deque<std::pair<float, std::chrono::milliseconds>> setpoints;
        uint16_t setpointType;
        int setLen = 0;
        int setPos = -1;
        bool cmdRun = false;
        std::vector<std::pair<float,uint16_t>> gainsAndCmd;
        std::string gainNames[6] {"p", "i", "d", "vel", "accel", "jerk"};
        std::chrono::steady_clock::time_point timer = std::chrono::steady_clock::now();
};

class PackageRelaxCommand : public Command {
    public:
        PackageRelaxCommand(PackageConfig &config, json &args):
            Command(config.id), config(config) {
                std::string type = args["type"];
                if(type.compare("relax") != 0) {
                    throw std::runtime_error("package relax command not of type relax");
                }
                relax = args["relax"];
            }
        std::pair<int, uint8_t*> interpret() { 
            return interpRelaxCommand(config.actuator.getAddr(), config.actuator.getPkgAddr(), relax);
        }
        std::string getVerb() { return "move"; }
        std::vector<uint32_t> getAssociatedFullAddrs() {
            auto v = std::vector<uint32_t>();
            // just use the actuator, feedback commands can still happen
            // in the background
            v.push_back(config.actuator.getFullAddr());
            return v;
        }
        ~PackageRelaxCommand() {};
    private:
        PackageConfig &config;
        bool relax;
};

// Allows other commands to sample a device with a given sampling interval and rate
// Use the exposed cmdDone boolean to control when to interpret new commands
// and bind the command's step/getTotalSteps and aggregateResponses to the corresponding commands
class CommandSampler {
    public:
        CommandSampler(json &args);
        bool step();
        int getTotalSteps();
        void aggregateResponses(json &acc, json &res, 
            int resNumber, std::function<float(float)> transResponse);
        bool cmdDone = false;
        int getStep() { return steps; }
    private:
        std::chrono::milliseconds sampleInterval = std::chrono::milliseconds(0);
        std::chrono::milliseconds sampleRate = std::chrono::milliseconds(0);
        std::chrono::steady_clock::time_point timer;
        int totalSteps = 0;
        int steps;
        bool sampling = false;
};

class SensorCommand : public Command {
    public:
        // takes in physical sensor id
        SensorCommand(FeedbackConfig &config, json &args);
        std::pair<int, uint8_t*> interpret();
        std::string getVerb() {
            return "query";
        }
        std::vector<uint32_t> getAssociatedFullAddrs() {
            auto v = std::vector<uint32_t>();
            v.push_back(config.getFullAddr());
            return v;
        }
        bool step(std::chrono::microseconds loopTime);
        int getTotalSteps();
        void aggregateResponses(json &acc, json &res, int resNumber);
        ~SensorCommand() { }
    private:
        FeedbackConfig &config;
        CommandSampler sampler;
};

class EyeCommand : public Command {
    public:
        EyeCommand(EyeConfig &config, json &args);
        std::pair<int, uint8_t*> interpret();
        std::string getVerb() {
            return "move";
        }
        std::vector<uint32_t> getAssociatedFullAddrs() {
            auto v = std::vector<uint32_t>();
            v.push_back(config.getFullAddr());
            return v;
        }
        ~EyeCommand() { }
    private:
        std::string type;
        uint8_t x1, y1, x2, y2, frames;
        EyeConfig &config;
};

class BaseArmCommand : public Command {
    public:
        BaseArmCommand(ArmConfig &config): 
            Command(config.id), config(config) {};
        virtual std::vector<uint32_t> getAssociatedFullAddrs() {
            auto v = std::vector<uint32_t>();
            // Add full addresses of package actuators associated
            for(auto pkg : config.jointPackages) {
                v.push_back(pkg->actuator.getFullAddr());
            }
            return v;
        }
        virtual std::string getVerb() = 0;
        virtual std::pair<int, uint8_t*> interpret() = 0;
        virtual ~BaseArmCommand() {};
    private:
        ArmConfig &config;
};

class ArmCommand : public BaseArmCommand {
    public:
        ArmCommand(ArmConfig &config, json &args);
        std::pair<int, uint8_t*> interpret();
        std::string getVerb() {
            return "move";
        }
        bool step(std::chrono::microseconds loopTime);
        int getTotalSteps() { return setpoints.size() * joints; }
        ~ArmCommand();
    private:
        ArmConfig &config;
        // Transforms the endpoint coordinates (X, Y, Z)
        // to joint coordinates and outputs an array containing them
        double *solveIk(double *endpoint);
        double ikSolverIter(double *testVars, double *endpoint);
        void matChainSubs(double *testVars, std::vector<SymTransMatrix*> &chain, 
            TransMatrix &curMat);
        // retrieves the current task space position of the motors
        std::tuple<double,double,double> getTaskPos();
        // processes json data into joint space positions
        // stores joint space data in setpoints
        // stores time data in waitTimes
        void processArguments(json &data, int joints, bool ik);
        std::vector<double*> setpoints;
        bool ik;
        int joints;
        int curStep = -1;
        int curJoint = 0;
        std::chrono::steady_clock::time_point timer;
        std::vector<std::chrono::milliseconds> waitTimes;
        std::vector<float*> velocities;
        bool moveDone = false;
        std::default_random_engine gen;
        std::uniform_real_distribution<double> uniformRand = 
            std::uniform_real_distribution<double>(0, 1);
        std::uniform_real_distribution<double> annealRand = 
            std::uniform_real_distribution<double>(-0.03, 0.03);
};

// TODO: factor out the main logic of this class because it's a massive pain to implement
// multiple queries at once due to race conditions
class ArmQueryCommand : public BaseArmCommand {
    public:
        ArmQueryCommand(ArmConfig &config, json &args);
        std::pair<int, uint8_t*> interpret();
        std::string getVerb() {
            return "query";
        }
        bool step(std::chrono::microseconds loopTime);
        int getTotalSteps() { return config.jointPackages.size() * sampler.getTotalSteps(); }
        void aggregateResponses(json &acc, json &res, int resNumber);
        std::vector<uint32_t> getAssociatedFullAddrs() {
            auto v = std::vector<uint32_t>();
            // Add full addresses of package feedbacks associated
            for(auto pkg : config.jointPackages) {
                v.push_back(pkg->feedback->getFullAddr());
            }
            return v;
        }
        ~ArmQueryCommand() {};
    private:
        ArmConfig &config;
        CommandSampler sampler;
        int curJoint = -1;
        bool jointSpace;
        bool cmdDone = false;
        std::set<int> recvJoint;
        int recvCmd = 0;
        void jointToTask(std::vector<double> &jointAngles, int idx, json &acc);
};

class ArmRelaxCommand : public BaseArmCommand {
    public:
        ArmRelaxCommand(ArmConfig &config, json &args);
        std::pair<int, uint8_t*> interpret();
        std::string getVerb() {
            return "move";
        }
        bool step(std::chrono::microseconds loopTime);
        int getTotalSteps() { return joints; }
        ~ArmRelaxCommand() {};
    private:
        ArmConfig &config;
        int joints;
        int curJoint = -1;
        bool relax;
};

#endif