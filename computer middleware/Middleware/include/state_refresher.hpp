#include <map>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include "command.hpp"
#include "state_sender.hpp"
#include "build_consts.hpp"

#ifndef STATE_REFRESH_H
#define STATE_REFRESH_H

class StateRefresher {
    public:
        StateRefresher(std::map<std::string, Command*> &executing,
            std::mutex &cmdsLock, StateSender &stateSender, std::chrono::nanoseconds rate): 
            executing(executing), cmdsLock(cmdsLock), stateSender(stateSender), rate(rate) {};
        // Refreshes the state of the connected microcontroller(s) and advances commands
        // accordingly, deleting them as necessary
        void refresh();
    private:
        std::map<std::string, Command*> &executing;
        std::mutex &cmdsLock;
        StateSender &stateSender;
        std::chrono::nanoseconds rate;
        std::chrono::time_point<std::chrono::steady_clock> lastRefresh = 
            std::chrono::steady_clock::now();
        std::chrono::nanoseconds processingTimeAcc = std::chrono::nanoseconds(0);
        int processingSamples = 0;
         std::chrono::microseconds loopTime = std::chrono::microseconds(0);
};

#endif