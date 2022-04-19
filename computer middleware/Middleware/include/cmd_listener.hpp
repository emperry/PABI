#include <functional>
#include <thread>
#include <string>
#include <condition_variable>
#include <iostream>
#include <vector>
#include <queue>
#include <nlohmann/json.hpp>
#include <zmqpp/zmqpp.hpp>
#include "command.hpp"
#include "cmd_config.hpp"
#include "response.hpp"
#include "config_manager.hpp"
using json = nlohmann::json;

#ifndef COMMAND_LISTENER_H
#define COMMAND_LISTENER_H

class CommandListener {
    public:
        CommandListener(std::string bind, ConfigurationManager cfgManager):
            bind(bind), cfgManager(cfgManager) {};
        void subscribe(std::function<void(std::forward_list<Command*>)> callback);
        // Starts the worker thread listening to responses
        void start();
        // Stops this worker thread
        void stop(); 
    private:
        void zmqServer();
        void wakeupOnRecv(zmqpp::context &context);
        std::forward_list<std::string> &getAddrs(zmqpp::message &msg);
        std::pair<uint, std::forward_list<Command*>> genCommands(json j);
        void addMoveToList(json &args, std::forward_list<Command*> &list, CommandConfig *cfg);
        // worker thread for zmq
        std::thread *worker;
        bool runServer = false;
        std::string bind;
        ConfigurationManager cfgManager;
        // Callback functions to be notified when a new
        // command comes through (subscribers)
        std::vector<std::function<void(std::forward_list<Command*>)>> callbacks;
        // Queue of response messages going out
        std::queue<zmqpp::message*> responses;
        // Condition variable/mutex signaling that the zmq server has started
        std::mutex startLock;
        std::condition_variable startCond;
        // Mutex that locks the response queue
        std::mutex resLock;
        std::condition_variable recv;
};

#endif