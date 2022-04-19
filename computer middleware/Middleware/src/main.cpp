#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <map>
#include <nlohmann/json.hpp>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "../include/cmd_listener.hpp"
#include "../include/command.hpp"
#include "../include/state_refresher.hpp"
#include "../include/state_sender.hpp"
using json = nlohmann::json;

#include "../include/mat.hpp"
#include "../include/cubic_spline.hpp"

CommandListener *listener;
std::thread *refreshThread;
ConfigurationManager *cfgManager;
StateSender *stateSender;
std::forward_list<CommandConfig*> *devices;
std::map<std::string, std::pair<std::string,int>> *mcuAddrs;
bool run = true;

void cleanUp(int s) {
    std::cout << "Ending middleware, cleaning up..." << std::endl;
    listener->stop();
    run = false; // TODO: probably a better way to signal the thread to stop
    refreshThread->join();
    // cleanup before exiting, make sure the microcontrollers
    // are in a stable state, deconstruct all objects
    cfgManager->cleanUp();
    stateSender->closeSockets();
    delete listener;
    delete refreshThread;
    delete cfgManager;
    delete stateSender;
    delete devices;
    delete mcuAddrs;
    exit(0);
}

int main(int argc, char *argv[]) {
    // TODO: make this mcu configuration dynamic and from the database
    mcuAddrs = new std::map<std::string, std::pair<std::string,int>>();
    (*mcuAddrs)["main"] = std::pair<std::string,int>("192.168.50.10", 5050);
    //(*mcuAddrs)["main"] = std::pair<std::string,int>("127.0.0.1", 5050);
    // Create configuration manager 
    cfgManager = new ConfigurationManager("http://admin:password@localhost:5984", "msarf_config");
    try {
        cfgManager->generateConfigs();
    } catch(const std::exception &e) {
        std::cout << "Error parsing configuration data" << std::endl;
        std::cout << e.what() << std::endl;
        raise(SIGINT);
    }
    // Create command listener (ZeroMQ listening/parsing thread)
    listener = new CommandListener("tcp://*:5050", *cfgManager);
    // Create command structure, associated mutex
    auto executingCmds = std::map<std::string, Command*>();
    std::mutex cmdsLock;
    // Updates commands in the command structure, add as a listener
    // to the command listener
    auto updateCmds = [&cmdsLock, &executingCmds]
    (std::forward_list<Command*> cmds) mutable { 
        std::lock_guard<std::mutex> guard(cmdsLock);
        for(auto cmd : cmds) {
            std::string id = cmd->getId();
            uint8_t newPriority = cmd->getPriority();
            // Does a first pass over all sub commands to see if any
            // have a higher priority than the new command
            // if they have a higher priority then the new command is invalidated
            bool newCmdInvalid = false;
            for(auto subId : cmd->getAssociatedIds()) {
               auto curCmd = executingCmds[subId];
               if(curCmd != NULL) {
                   if(curCmd->getPriority() < newPriority) {
                       // Invalidate the new command instead of the sub commands
                        json j = {
                            {"error", "higher priority command in progress"}  
                        };
                        cmd->finish(j, true);
                        delete cmd;
                        newCmdInvalid = true;
                        break;
                   }
               }
            }
            if(newCmdInvalid) {
                continue;
            }
            // Second pass, invalidate sub commands
            for(auto subId : cmd->getAssociatedIds()) {
               auto curCmd = executingCmds[subId];
               if(curCmd != NULL) {
                    json j = {
                        {"error", "invalidated by another command"}  
                    };
                    curCmd->finish(j, true);
                    for(auto removeId : curCmd->getAssociatedIds()) {
                        executingCmds.erase(removeId);
                    }
                    delete curCmd;
               }
               executingCmds[subId] = cmd;
            }
        }
    };
    listener->subscribe(updateCmds);
    // Create the state refresher, run the refresh command continuously
    // the refresh command automatically handles timing
    stateSender = new StateSender(*mcuAddrs);
    devices = new std::forward_list<CommandConfig*>();
    for(auto cfg : cfgManager->getConfigMap()) {
        devices->push_front(cfg.second);
    }
    stateSender->initMcu("main", *devices);
    auto stateRefresher = StateRefresher(executingCmds, 
        cmdsLock, *stateSender, std::chrono::milliseconds(1));
    refreshThread = new std::thread([&stateRefresher]() mutable {
        while(run) {
            stateRefresher.refresh();
        }
    });
    listener->start();
    // Catch SIGINT from CTRL+C and clean up
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = cleanUp;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    pause();
}
