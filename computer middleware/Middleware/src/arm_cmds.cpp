
#include "../include/command.hpp"
#include "../include/cubic_spline.hpp"

/* Arm command */
ArmCommand::ArmCommand(ArmConfig &config, json &args):
        BaseArmCommand(config), config(config) {
    curStep = -1;
    curJoint = 0;
    // Make sure SymTransMatrix variables are present
    if(config.matrixChain.size() == 0) {
        throw std::invalid_argument("no matrix chain present, check configuration");
    }
    // Verify arguments are correct
    if(!args["data"].is_array()) {
        throw std::invalid_argument("no data field present");
    }
    std::string type = args["type"];
    if(type.compare("ik") == 0) {
        ik = true;
    } else if(type.compare("fk") == 0) {
        ik = false;
    } else {
        throw std::invalid_argument("invalid type argument, must be ik or fk");
    }
    auto data = args["data"];
    joints = config.jointPackages.size();
    // Full arm command procedure:
    // 1. need to get coordinates (task or joint) out of command
    // 2. need to turn ik coordinates into fk if needed
    // 3. clampIfOutside/other conditioning, add times if needed
    // 4. calculate joint times per endpoint
    // 5. interpolate joint space coordinates using cubic splines

    // processes the json data field using flags into
    // joint space setpoints and wait times in between them
    processArguments(data, joints, ik);

    // if there's only one setpoint then add a zero velocity and continue
    if(setpoints.size() == 1) {
        auto v = new float[joints];
        for(int i = 0; i < joints; i++) {
            v[i] = 0;
        }
        velocities.push_back(v);
    } else {

        // interpolate between these setpoints and calculate velocities:
        // use setpoints and wait times calculated above, then use
        // cubic spline interpolation between the joint space coordinates
        // and calculate intermediate times and velocities
        std::vector<double*> interpSetpoint;
        std::vector<std::chrono::milliseconds> waitTimesInterp;
        for(int i = 0; i < joints; i++) {
            // generate a vector for 1 joint and time
            std::vector<std::tuple<double,double>> jointParam;
            int64_t timeAcc = 1;
            for(uint j = 0; j < setpoints.size(); j++) {
                timeAcc += waitTimes[j].count();
                auto pt = std::make_tuple(timeAcc, setpoints[j][i]);
                jointParam.push_back(pt);
            }
            // interpolate for this one joint
            auto jointInterp = cubicSplineInterp(jointParam, 100);
            // put this into interpSetpoint, waitTimesInterp and velocities
            for(uint j = 0; j < jointInterp.size(); j++) {
                double t, x1;
                std::tie(t, x1) = jointInterp[j];
                if(interpSetpoint.size() <= j) {
                    interpSetpoint.push_back(new double[joints]);
                    double tm1, xm1 = 0;
                    // generate wait times if they haven't been already
                    // wait times are the differnece between this point and the last one
                    auto prevT = std::chrono::milliseconds(0);
                    if(interpSetpoint.size() > 1) {
                        std::tie(tm1, xm1) = jointInterp[j-1];
                        prevT = std::chrono::milliseconds((int)tm1);
                    }
                    waitTimesInterp.push_back(std::chrono::milliseconds((int)t-prevT.count()));
                    // Generate velocity
                    auto v = new float[joints];
                    for(int k = 0; k < joints; k++) {
                        if(interpSetpoint.size() == 1) {
                            xm1 = config.jointPackages[k]->actuator.getCurPos();
                        }
                        v[k] = config.jointPackages[k]->actuator
                            .calcVel(xm1, x1, waitTimesInterp.back());
                    }
                    velocities.push_back(v);
                }
                // put the current interpolated joint setpoint into the array
                interpSetpoint[j][i] = x1;
            }
        }
        
        // delete non interpolated setpoints
        for(uint i = 0; i < setpoints.size(); i++) {
            delete[] setpoints[i];
        }

        // replace setpoints with interpolated ones
        setpoints = interpSetpoint;
        // replace wait times with interpolated ones
        waitTimes = waitTimesInterp;
    }

    // DEBUG: print out times and setpoints
    for(uint i = 0; i < setpoints.size(); i++) {
        for(uint j = 0; j < (uint)joints; j++) {
            std::cout << j << ": " << setpoints[i][j] << " ";
        }
        std::cout << waitTimes[i].count() << std::endl;
    }
    
    timer = std::chrono::steady_clock::now();
}

void ArmCommand::processArguments(json &data, int joints, bool ik) {
    auto t = std::chrono::milliseconds(0);
    // the first point should be the current position
    auto pt = new double[joints];
    std::cout << "INITIAL POS: ";
    for(int i = 0; i < joints; i++) {
        pt[i] = config.jointPackages[i]->actuator.getCurPos();
        std::cout << pt[i] << " ";
    }
    std::cout << std::endl;
    setpoints.push_back(pt);
    waitTimes.push_back(t);
    double *curSetpoint;
    for(auto &row : data) {
        try {
            // convert to joint space coordinates if necessary
            if(ik) {
                if(!row.is_object()) {
                    throw std::invalid_argument("rows in data must be objects");
                }
                double x = row["x"], y = row["y"], z = row["z"];
                // check for NaN
                if(isnan(x) || isnan(y) || isnan(z)) {
                    throw std::runtime_error("tried to run move arm command with nan in arguments");
                }
                auto endpoint = new double[3] {x, y, z};
                std::cout << "X: " << x << " Y: " << y << " Z: " << z << std::endl;
                curSetpoint = solveIk(endpoint);
                delete[] endpoint;
                if(row["t"].is_number()) {
                    t = std::chrono::milliseconds((int64_t)row["t"]);
                }
            } else {
                if(!row.is_array()) {
                    throw std::invalid_argument("rows in data must be arrays");
                }
                if((int)row.size() > joints + 1) {
                    throw std::invalid_argument("row has an improper amount of arguments");
                }
                curSetpoint = new double[joints];
                for(int i = 0; i < joints; i++) {
                    if(isnan((double)row[i])) {
                        delete[] curSetpoint;
                        throw std::runtime_error("tried to run move arm command with nan in arguments");
                    }
                    curSetpoint[i] = config.jointPackages[i]->
                        actuator.clampIfOutside(row[i]);
                }
                if((int)row.size() > joints && row[joints].is_number()) {
                    t = std::chrono::milliseconds((int64_t)row[joints]);
                }
            }
        } catch(std::exception &e) {
            for(auto s : setpoints) {
                delete[] s;
            }
            throw;
        }
        // calculate time to get to next point and add to the current 't'
        // find the longest time a joint has to take to reach its destination
        // and then make every other joint's velocity less to reach it at the same time
        auto maxTime = std::chrono::milliseconds(0);
        for(int i = 0; i < joints; i++) {
            auto &curActuator = config.jointPackages[i]->actuator;
            auto pos = curActuator.getCurPos();
            if(setpoints.size() > 0) {
                pos = setpoints.back()[i];
            }
            auto timeToNext = curActuator.calcTime(pos, curSetpoint[i]);
            if(maxTime < timeToNext) {
                maxTime = timeToNext;
            }
        }
        std::cout << "MOVE TO SETPOINT: ";
        for(int i = 0; i < joints; i++) {
            std::cout << curSetpoint[i] << " ";
        }
        std::cout << std::endl;
        // the max time is the total time it will take to do the given action
        t += maxTime;
        std::cout << "MOVE WITH TIME: " << t.count() << std::endl;

        // if this setpoint is equal to the last, then drop it
        bool equal = true;
        for(int i = 0; i < joints; i++) {
            if(curSetpoint[i] != setpoints.back()[i]) {
                equal = false;
                break;
            }
        }
        if(equal) {
            delete[] curSetpoint;
            continue;
        }

        // store setpoints and waitTimes
        setpoints.push_back(curSetpoint);
        waitTimes.push_back(t);
    }
}

std::tuple<double,double,double> ArmCommand::getTaskPos() {
    double *jointVars = new double[config.jointPackages.size()];
    for(uint i = 0; i < config.jointPackages.size(); i++) {
        ActuatorConfig &curActuator = config.jointPackages[i]->actuator;
        double curPos = curActuator.getCurPos();
        curPos = curActuator.reverseScaleAndZero(curPos);
        std::cout << curPos << std::endl;
        jointVars[i] = curPos;
    }
    TransMatrix a;
    matChainSubs(jointVars, config.matrixChain, a);
    delete[] jointVars;
    auto task = a.getTranslate();
    auto taskTuple = std::make_tuple(task[0], task[1], task[2]);
    delete[] task;
    return taskTuple;
}

// Debug for when you want to display all of the steps the IK algorithm takes for graphing
//#define DISP_STEPS

// IK debug on or off
#define IK_DEBUG

double *ArmCommand::solveIk(double *endpoint) {
    // Use a heuristic to compute the joint values iteratively
    // Stop when it has converged or the time alotted to the calculation
    // has expired
    std::vector<std::string> varList;
    for(auto v : config.matVars) {
        varList.push_back(v);
    }
    auto testVars = new double[varList.size()];
    auto ongoingVars = new double[varList.size()];
    // starting joint positions
    for(uint i = 0; i < varList.size(); i++) {
        ongoingVars[i] = M_PI_4;
        //ongoingVars[i] = 0;
        //ongoingVars[i] = config.jointPackages[i]->actuator.getCurPos();
    }
    double prevErr = ikSolverIter(ongoingVars, endpoint); // previous error (simulated annealing)
    double t; // temperature (simulated annealing)
    double err = 0; // error
    auto timeLimit = std::chrono::milliseconds(30);
    #ifdef IK_DEBUG
    uint k = 0; // iteration counter
    std::chrono::steady_clock::time_point endTime;
    bool found = false;
    #endif
    #ifdef DISP_STEPS
    std::vector<double*> steps;
    #endif
    auto startTime = std::chrono::steady_clock::now();
    while((std::chrono::steady_clock::now() - startTime) < timeLimit) {
        // compute temperature (exponentially decreasing from 1 to 0 across time)
        double duration = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - startTime).count() / 1000.0;
        // https://www.desmos.com/calculator/8z30kbijnj
        t = exp((log(0.0001)/timeLimit.count())*duration);
        // randomly change variables
        for(uint i = 0; i < varList.size(); i++) {
            testVars[i] = ongoingVars[i] + annealRand(gen);
        }
        err = ikSolverIter(testVars, endpoint);
        // Determine weather or not to take this change
        bool accepted = false;
        double change = err - prevErr;
        // Change if needed
        if(change < 0) {
            // take the change
            accepted = true;
        } else {
            // take the change based on probability
            double prob = exp(-change/t);
            double rand = uniformRand(gen);
            accepted = rand <= prob;
        }
        if(accepted) {
            #ifdef DISP_STEPS
            steps.push_back(testVars);
            ongoingVars = testVars;
            testVars = new double[varList.size()];
            #else
            delete[] ongoingVars;
            ongoingVars = testVars;
            testVars = new double[varList.size()];
            #endif
        }
        if(err < 1) {
            #ifdef IK_DEBUG
            std::cout << "IK SOLUTION FOUND" << std::endl;
            endTime = std::chrono::steady_clock::now();
            found = true;
            #endif
            break;
        }
        prevErr = err;
        #ifdef IK_DEBUG
        k++;
        #endif
    }
    #ifndef DISP_STEPS
    delete[] testVars;
    #else
    for(auto step : steps) {
        for(uint i = 0; i < varList.size(); i++) {
            std::cout << step[i] << " ";
        }
        std::cout << ",";
    }
    std::cout << std::endl;
    #endif
    #ifdef IK_DEBUG
    std::cout << "TOOK " << k << " STEPS FOR IK, AVG LOOP TIME: " << (double)timeLimit.count()/k << "ms" << std::endl;
    if(found) {
        std::cout << "DURATION: " << 
            std::chrono::duration_cast<std::chrono::milliseconds>
                (endTime - startTime).count() << "ms"
            << std::endl; 
    } else {
        std::cout << "EXHAUSTED TIME" << std::endl;
    }
    std::cout << "IK ANGLES: ";
    #endif
    for(uint i = 0; i < varList.size(); i++) {
        auto cur = varList[i];
        while(ongoingVars[i] > M_PI * 2) {
            ongoingVars[i] -= M_PI * 2;
        }
        #ifdef IK_DEBUG
        std::cout << cur << " " << ongoingVars[i] << " ";
        #endif
        ongoingVars[i] = config.jointPackages[i]->
            actuator.clampIfOutside(ongoingVars[i]);
    }
    #ifdef IK_DEBUG
    std::cout << std::endl;
    std::cout << "FINAL IK ERROR: " << err << std::endl;
    #endif
    return ongoingVars;
}

double ArmCommand::ikSolverIter(double *testVars, double *endpoint) {
    TransMatrix curMat;
    // substitute into the matrix chain and find the
    // endpoint value at our current guess
    matChainSubs(testVars, config.matrixChain, curMat);
    auto curEndpoint = curMat.getTranslate();
    //std::cout << curEndpoint[0] << " " << curEndpoint[1] << " " << curEndpoint[2] << std::endl;
    // find the difference between the guess and the goal
    // using the distance formula
    double err = 0;
    for(int i = 0; i < 3; i++) {
        err += pow(curEndpoint[i] - endpoint[i], 2);
    }
    delete[] curEndpoint;
    err = sqrt(err);
    return err;
}

void ArmCommand::matChainSubs(double *testVars, 
        std::vector<SymTransMatrix*> &chain, 
        TransMatrix &curMat) {
    //std::cout << "----" << std::endl;
    //for(auto i = chain.rbegin(); i != chain.rend(); i++) {
    //    auto mat = *i;
    // current sym position
    int i = 0; // TODO: assumes each sym occurs only once, maybe fix this?
    for(auto mat : chain) {
        if(mat->getSymName().compare("") == 0) {
            auto m = mat->substitute(NULL);
            //m.print();
            curMat = curMat.mult(m);
        } else {
            auto m = mat->substitute(&testVars[i]);
            i++;
            //m.print();
            curMat = curMat.mult(m);
        }
    }
}

std::pair<int, uint8_t*> ArmCommand::interpret() {
    if(moveDone) {
        return std::pair<int, uint8_t*>(0, NULL);
    }
    //std::cout << curJoint << " " << curStep << std::endl;
    // Retrieve the current joint and current setpoint, then send
    // the appropriate command
    auto &joint = config.jointPackages[curJoint]->actuator;
    auto setpoint = (float)setpoints[curStep][curJoint];
    float curVFloat = std::abs(velocities[curStep][curJoint]);
    int32_t curV = 0;
    memcpy(&curV, &curVFloat, 4);
    uint16_t addr = joint.getAddr();
    uint8_t pkgAddr = joint.getPkgAddr();
    int32_t arg;
    memcpy(&arg, &setpoint, 4);
    int size = 15;
    auto *cmd = new uint8_t[size];
    cmd[0] = OP_COMMAND;
    cmd[1] = addr >> 8;
    cmd[2] = addr;
    cmd[3] = pkgAddr;
    cmd[4] = CMD_POSITION >> 8;
    cmd[5] = CMD_POSITION;
    cmd[6] = 0x8;
    cmd[7] = arg >> 24;
    cmd[8] = arg >> 16;
    cmd[9] = arg >> 8;
    cmd[10] = arg;
    cmd[11] = curV >> 24;
    cmd[12] = curV >> 16;
    cmd[13] = curV >> 8;
    cmd[14] = curV;
    moveDone = true;
    return std::pair<int, uint8_t*>(size, cmd);
}

bool ArmCommand::step(std::chrono::microseconds loopTime) {
    bool atEnd = curStep == (int)setpoints.size();
    if(curStep != -1) {
        // interpolate with velocity (this doesn't seem to work with cubic spline interp)
        /*for(int i = 0; i < joints; i++) {
            // display interpolated joint positions
            //std::cout << i << ": " << config.jointPackages[i]->actuator.curPos << std::endl;
            auto idx = atEnd ? curStep-1 : curStep;
            auto setpoint = setpoints[idx][i];
            auto velocity = velocities[idx][i];
            auto seconds = ((float)loopTime.count())/1e6;
            auto curPos = config.jointPackages[i]->actuator.getCurPos();
            // interpolate joint positions (only if we aren't already there)
            if(std::abs(curPos-setpoint) > 0.001) {
                config.jointPackages[i]->actuator.setCurPos(curPos + (velocity * seconds));
            }
        }*/
    }
    //std::cout << std::endl;
    if(atEnd && (std::chrono::steady_clock::now() - timer) > waitTimes[curStep-1]) {
        for(int i = 0; i < joints; i++) {
            // set the motor position internally
            config.jointPackages[i]->actuator.setCurPos(setpoints[curStep-1][i]);
            //std::cout << "2MOVED: " << i << " TO: " << curStep << std::endl;
        }
        return false;
    } else if(atEnd) {
        return true;
    }

    if(curStep == -1) {
        curStep++;
        return true;
    }
    if(curStep < (int)setpoints.size()) {
        if(curJoint < joints - 1) {
            curJoint++;
            moveDone = false;
        }
        if(curJoint == joints - 1 && moveDone &&
          (std::chrono::steady_clock::now() - timer) > waitTimes[curStep]) {
            for(int i = 0; i < joints; i++) {
                // set the motor position internally
                config.jointPackages[i]->actuator.setCurPos(setpoints[curStep][i]);
                //std::cout << "1MOVED: " << i << " TO: " << curStep << std::endl;
            }
            curStep++;
            curJoint = 0;
            timer = std::chrono::steady_clock::now();
            if(curStep != (int)setpoints.size()) {
                moveDone = false;
            }
        }
        //return curStep < (int)setpoints.size();
    }
    return true;
}

ArmCommand::~ArmCommand() {
    for(auto s : setpoints) {
        delete[] s;
    }
    for(auto v : velocities) {
        delete[] v;
    }
}

ArmQueryCommand::ArmQueryCommand(ArmConfig &config, json &args):
    BaseArmCommand(config), config(config), sampler(args) {
        std::string type = args["type"];
        if(type.compare("joint") == 0) {
            jointSpace = true;
        } else if(type.compare("task") == 0) {
            jointSpace = false;
        } else {
            throw std::invalid_argument("invalid type argument, must be joint or task space");
        }
    }

std::pair<int, uint8_t*> ArmQueryCommand::interpret() {
    if(cmdDone || recvCmd < sampler.getStep()) {
        //std::cout << "BLOCKED2" << std::endl;
        return std::pair<int, uint8_t*>(0, NULL);
    }
    std::cout << "I" << " J: " << curJoint << std::endl;
    auto curSensor = config.jointPackages[curJoint]->feedback;
    auto addr = curSensor->getAddr();
    auto pkgAddr = curSensor->getPkgAddr();
    return interpSensorCommand(addr, pkgAddr, cmdDone);
}

// This assumes that all responses come after the command is sent
// and that the responses return in order of when the commands were sent
void ArmQueryCommand::aggregateResponses(json &acc, json &res, int resNumber) {
    if(!acc["data"].is_array()) {
        acc["data"] = {};
    }
    if(res["response"].is_number()) {
        auto joints = config.jointPackages.size();
        int lastIdx = acc["data"].size() - 1;
        int curJointIdx = -1;
        for(uint i = 0; i < joints; i++) {
            if(config.jointPackages[i]->feedback->getFullAddr() == res["fullAddr"]) {
                curJointIdx = i;
                break;
            }
        }
        if(curJointIdx == -1) {
            throw std::logic_error("joint not found for arm query command");
        }
        auto curJointPkg = config.jointPackages[curJointIdx];
        // if this is the first response, or we need to create a new array
        // to hold the next joints
        if(lastIdx == -1 || recvJoint.size() == joints) {
            if(!jointSpace && recvJoint.size() == joints) {
                std::vector<double> jointAngles = acc["data"][lastIdx];
                jointToTask(jointAngles, lastIdx, acc);
            }
            acc["data"].push_back(json::array());
            lastIdx++;
            // reset the recieve counter
            recvJoint = std::set<int>();
        }
        /*if(recvJoint.count(curJointIdx)) {
            std::cout << "recieved joint twice for arm query" << std::endl;
        }*/
        // counts all of the joints we have recieved for the given sample iteration
        // set must have all joint values before continuing
        // this is because if this command is overriden then there may be extraneous
        // commands from the previous time it was run
        recvJoint.insert(curJointIdx);
        //std::cout << lastIdx << " " << recv << std::endl;
        std::cout << "R " << recvJoint.size() << " " << joints << " " << curJointIdx << std::endl;
        if(recvJoint.size() == joints) {
            std::cout << "DONE" << std::endl;
            recvCmd++;
        }
        //std::cout << "I:" << lastIdx << std::endl;
        float curRes = res["response"];
        acc["data"][lastIdx][curJointIdx] = 
            curJointPkg->feedback->scaleAndZero(curRes);
        if(!jointSpace && lastIdx == sampler.getTotalSteps()-1 && recvJoint.size() == joints) {
            std::vector<double> jointAngles = acc["data"][lastIdx];
            jointToTask(jointAngles, lastIdx, acc);
        }
    }
}

void ArmQueryCommand::jointToTask(std::vector<double> &jointAngles, int idx, json &acc) {
    // multiply matrices to find task space endpoint
    // for the previous positions
    int angIdx = 0;
    auto curMat = TransMatrix();
    for(auto m : config.matrixChain) {
        if(m->getSymName().compare("") == 0) {
            auto newM = m->substitute(NULL);
            curMat = curMat.mult(newM);
        } else {
            double ang = jointAngles[angIdx];
            //ang = config.jointPackages[angIdx]->feedback->scaleAndZero(ang);
            auto newM = m->substitute(&ang);
            curMat = curMat.mult(newM);
            angIdx++;
        }
    }
    // replace it in the response object
    auto t = curMat.getTranslate();
    acc["data"][idx] = {{"x", t[0]}, {"y", t[1]}, {"z", t[2]}};
    delete[] t;
}

bool ArmQueryCommand::step(std::chrono::microseconds loopTime) {
    if(recvCmd < sampler.getStep()) {
        //std::cout << "BLOCKED1" << std::endl;
        return true;
    }
    int joints = config.jointPackages.size();
    if(curJoint == joints-1) {
        auto steps = sampler.getStep();
        sampler.step();
        if(steps < sampler.getStep()) {
            curJoint = 0;
            cmdDone = false;
        } else {
            cmdDone = true;
        }
    } else {
        curJoint++;
        cmdDone = false;
    }
    std::cout << "S " << sampler.getStep() << " " << cmdDone << " " << curJoint << std::endl;
    return (int)sampler.getTotalSteps() > recvCmd;
}

// Relax arm command
ArmRelaxCommand::ArmRelaxCommand(ArmConfig &config, json &args):
    BaseArmCommand(config), config(config), joints(config.jointPackages.size()) {
        std::string type = args["type"];
        if(type.compare("relax") != 0) {
            throw std::runtime_error("not a relax command");
        }
        relax = args["relax"];
    }

std::pair<int, uint8_t*> ArmRelaxCommand::interpret() {
    auto &act = config.jointPackages[curJoint]->actuator;
    return interpRelaxCommand(
        act.getAddr(),
        act.getPkgAddr(),
        relax
    );
}

bool ArmRelaxCommand::step(std::chrono::microseconds loopTime) {
    if(curJoint < joints-1) {
        curJoint++;
        return true;
    }
    return false;
}