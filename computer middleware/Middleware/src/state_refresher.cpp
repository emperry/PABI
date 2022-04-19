#include "../include/state_refresher.hpp"

void StateRefresher::refresh() {
    auto start = std::chrono::steady_clock::now();
    auto cmds = std::forward_list<std::pair<int, uint8_t*>>();
    std::unique_lock<std::mutex> guard(cmdsLock);
    // refresh state
    int stepAmt = ((start - lastRefresh) / rate) + 1;
    if(stepAmt > 1) {
        std::cout << "Missing " << (stepAmt - 1) << " steps" << std::endl;
    }
    // TODO: maybe we don't need to generate fullAddrToId every time?
    auto fullAddrToId = std::map<uint32_t, std::string>();
    // pairs to remove from executing after iterating (cannot iterate and remove at the same time)
    // these are added when all the command responses have been recieved but the response
    // is still being held back for other reasons (actuators need to move)
    std::vector<std::string> executingToRemove;
    for(std::pair<std::string, Command*>  pair : executing) {
        // step the commands
        auto newStep = pair.second->step(loopTime);
        pair.second->setLastStep(newStep);
        if(pair.second->finishIfEnded()) {
            executingToRemove.push_back(pair.first);
            continue;
        }
        // interpret the command and put it into the interpreted commands list
        // if there are more steps to the command
        if(newStep) {
            pair.second->resetTimer();
            auto cmd = pair.second->interpret();
            if(cmd.first > 0) {
                cmds.push_front(cmd); 
            }
        }
        for(auto fullAddr : pair.second->getAssociatedFullAddrs()) {
            fullAddrToId[fullAddr] = pair.first;
        }
    }
    // remove pairs that have been finished
    for(auto id : executingToRemove) {
        delete executing[id];
        executing.erase(id);
    }
    // send the commands
    // TODO: make the microcontroller configurable
    stateSender.send("main", cmds);
    // Pass in a map of combined addresses to string command ids
    // so they can be back referenced to current commands
    auto resData = stateSender.pollRecieve("main", fullAddrToId);
    // remove commands that are completed
    for(auto res : resData) {
        // run their callbacks with the response data
        for(auto resJson : res.second) {
            auto id = res.first;
            if(executing[id] != NULL) {
                auto cmd = executing[id];
                auto finished = cmd->finish(resJson, false);
                if(finished) {
                    #ifdef DEBUG
                    std::cout << "SENDING RES" << std::endl;
                    #endif
                    for(auto eraseId : cmd->getAssociatedIds()) {
                        executing.erase(eraseId);
                    }
                    delete cmd;
                }
            } else {
                std::cout << "Received response to id: " << id 
                <<  ", but no command was sent, ignoring" << std::endl;
            }
        }
    }
    // TODO: maybe make this more efficient?
    // if a command hasn't had a response in a certain amount of time
    // then give up on the command and return an error message
    auto newExecuting = std::map<std::string, Command*>();
    for(auto pair : executing) {
        if(pair.second != NULL) {
            if(pair.second->isTimedOut()) {
                // Timed out, send an error command back
                json err = {
                    {"error", "response not recieved in time"}
                };
                pair.second->finish(err, true);
                delete pair.second;
            } else {
                newExecuting[pair.first] = pair.second;
            }
        }
    }
    executing = newExecuting;
    guard.unlock();
    auto end = std::chrono::steady_clock::now();
    auto timeProcessing = end - start;
    std::this_thread::sleep_for(rate - timeProcessing);
    lastRefresh = std::chrono::steady_clock::now();
    loopTime = std::chrono::duration_cast<std::chrono::microseconds>(lastRefresh - start);
    #ifdef PRINT_LOOP_TIME
    std::cout << "Processing: " << (end - start).count() 
        << " Total Time: " << (lastRefresh - start).count() << std::endl;
    #endif
}