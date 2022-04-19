#include <string>
#include <condition_variable>
#include <iostream>
#include <vector>
#include <queue>
#include <nlohmann/json.hpp>
#include <zmqpp/zmqpp.hpp>
using json = nlohmann::json;

#ifndef RESPONSE_H
#define RESPONSE_H

class Command;

class Response {
    public:
        Response(std::forward_list<std::string> &addrs, uint size,
                std::queue<zmqpp::message*> &queue, 
                std::condition_variable &sendrecv, 
                std::mutex &resLock,
                uint64_t id) 
                : addrs(addrs), size(size), queue(queue), 
                  sendrecv(sendrecv), resLock(resLock), id(id) {
                      curSize = 0;
                  };
        // Callback function that records
        // the response to the command and adds the messages
        // to the queue once done. Returns weather this was the last
        // command in the chain or not.
        bool commandFinished(Command *cmd, json &msg);
        ~Response();
    private:  
        uint curSize;
        std::forward_list<std::string> &addrs;
        uint size;
        std::queue<zmqpp::message*> &queue;
        std::condition_variable &sendrecv;
        std::mutex &resLock;
        // lock for when a command finishes
        std::mutex updateLock;
        json res;
        uint64_t id;
};

#endif