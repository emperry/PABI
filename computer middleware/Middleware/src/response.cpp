#include "../include/response.hpp"
#include "../include/command.hpp"

bool 
Response::commandFinished(Command *cmd, json &msg) {
    // Lock each response because not everything here is atomic/thread safe
    std::lock_guard<std::mutex> l(updateLock);
    res[cmd->getVerb()][cmd->getId()] = msg;
    curSize++;
    // generate message, send response
    if(curSize == size) {
        auto *resMsg = new zmqpp::message();
        for(auto addr : addrs) {
            *resMsg << addr;
        }
        *resMsg << "";
        res["id"] = id;
        *resMsg << res.dump();
        // get lock
        std::unique_lock<std::mutex> lock(resLock);
        // add to queue
        queue.push(resMsg);
        // signal to send/recv
        lock.unlock();
        sendrecv.notify_one();
        return true;
    }
    return false;
}

Response::~Response() {
    addrs.clear();
    res.clear();
    delete &addrs;
}