#include "../include/cmd_listener.hpp"
using json = nlohmann::json;

void CommandListener::subscribe(
std::function<void(std::forward_list<Command*>)> callback) {
    callbacks.push_back(callback);
}

void CommandListener::start() {
    if(!runServer) {
        runServer = true;
        worker = new std::thread(&CommandListener::zmqServer, this);
        std::unique_lock<std::mutex> sl(startLock);
        startCond.wait(sl);
    } else {
        // Thread is already executing
        std::cout << "Command listener thread is already running..." << std::endl;
    }
}

void CommandListener::stop() {
    if(runServer) {
        // signal ending the server
        runServer = false;
        // end thread gracefully
        {
            std::lock_guard<std::mutex> lock(resLock);
            recv.notify_one();
        }
        worker->join();
        delete worker;
    } else {
        // No thread running
        std::cout << "No command listener thread running..." << std::endl;
    }
}

std::forward_list<std::string> &
CommandListener::getAddrs(zmqpp::message &msg) {
    auto addrs = new std::forward_list<std::string>();
    // retrieve addresses, the end delimiter is a 0 frame
    std::string addr;
    do {
        msg >> addr;
        if(addr.compare("") != 0) {
            addrs->push_front(addr);
        }
    } while(addr.compare("") != 0 && msg.remaining() > 0);
    return *addrs;
}

void CommandListener::wakeupOnRecv(zmqpp::context &context) {
    zmqpp::socket recvWakeup(context, zmqpp::socket_type::pair);
    recvWakeup.connect("inproc://onrecv");
    while(runServer) {
        std::unique_lock<std::mutex> recvLock(resLock);
        recv.wait(recvLock);
        recvWakeup.send(zmqpp::signal::ok);
    }
    recvWakeup.close();
}

void CommandListener::zmqServer() {
    std::cout << "Starting command listener..." << std::endl;
    zmqpp::context context;
    zmqpp::socket socket(context, zmqpp::socket_type::router);
    std::cout << "Binding to " << bind << "..." << std::endl;
    socket.bind(bind);
    zmqpp::socket recvWakeup(context, zmqpp::socket_type::pair);
    // bind to inproc
    recvWakeup.bind("inproc://onrecv");
    // start up the recieve wakeup thread
    std::thread wakeup(&CommandListener::wakeupOnRecv, this, std::ref(context));
    zmqpp::poller sendRecvPoller;
    sendRecvPoller.add(socket);
    sendRecvPoller.add(recvWakeup);
    {
        std::lock_guard<std::mutex> sl(startLock);
    }
    startCond.notify_one();
    while(runServer) {
        zmqpp::message message;
        if(socket.receive(message, true)) {
            auto &addrs = getAddrs(message);
            // try to parse the JSON, if not respond with an error
            std::string text;
            if(message.remaining() == 0) {
                std::cout << "Recieved message that has no data, failing" << std::endl;
                text = "";
            } else {
                message >> text;
            }
            json j;
            try {
                j = json::parse(text);
                auto cmdPair = genCommands(j);
                auto cmds = cmdPair.second;
                uint size = cmdPair.first;
                uint64_t passId = 0;
                uint8_t priority = 0;
                // pass through id for routing purposes
                if(j["id"].is_number_unsigned()) {
                    passId = j["id"];
                }
                // pass through priority
                if(j["priority"].is_number_unsigned()) {
                    priority = j["priority"];
                }
                auto *res = new Response(addrs, size, responses, recv, resLock, passId);
                for(auto cmd : cmds) {
                    cmd->setPriority(priority);
                    cmd->addCallback(*res);
                }
                for(auto func : callbacks) {
                    func(cmds);
                }
            } catch(std::exception &e) {
                std::cout << "Invalid message " << text << std::endl;
                std::cout << e.what() << std::endl;
                uint64_t passId = 0;
                bool passIdFound = false;
                if(j["id"].is_number_unsigned()) {
                    passId = j["id"];
                    passIdFound = true;
                }
                // Respond with an error
                zmqpp::message res;
                for(auto addr : addrs) { // TODO: repeated code, how to factor this out?
                    res << addr;
                }
                std::string exceptionMsg(e.what());
                res << "";
                 if(!passIdFound) {
                    res << "{\"error\": \"" + exceptionMsg + "\"}";
                 } else {
                     res << "{\"error\": \"" + exceptionMsg + "\",\"id\":"+std::to_string(passId)+"}";
                 }
                socket.send(res, true);
                delete &addrs;
            }
        }
        {
            std::lock_guard<std::mutex> lock(resLock);
            // send pending messages
            while(responses.size() != 0) {
                zmqpp::message *res = responses.front();
                responses.pop();
                socket.send(*res, true);
                delete res;
            }
            // dump out all the signals
            zmqpp::signal sig;
            while(recvWakeup.receive(sig, true)) {}
        }
        // wait for a message to send or a message to recieve
        sendRecvPoller.poll(1000); // TODO: stop spin locking every second
    }
    wakeup.join();
    recvWakeup.close();
    socket.close();
}

#define ITER_VERB(VERB) \
    if(j[VERB].is_object()) \
        for (auto it = j[VERB].begin(); it != j[VERB].end(); ++it)

std::pair<uint, std::forward_list<Command*>> 
CommandListener::genCommands(json j) {
    auto list = std::forward_list<Command*>();
    uint size = 0;
    // TODO: auto verb somehow? How can we automatically make new commands?
    ITER_VERB("move") {
        size++;
        // Use the key to look up the config, then use the type
        // attribute to cast to the correct type. Use the config to
        // create a command and push it into the list.
        CommandConfig *cfg = cfgManager.getConfigMap()[it.key()];
        if(!cfg) {
            std::cout << "Invalid command id " << it.key() << std::endl;
            throw std::invalid_argument("Invalid command id:" + it.key());
        }
        json args = j["move"][it.key()];
        addMoveToList(args, list, cfg);
    }
    ITER_VERB("query") { 
        size++;
        // Use key to look up sensor config and then make a sensor command
        CommandConfig *cfg = cfgManager.getConfigMap()[it.key()];
        if(!cfg) {
            throw std::invalid_argument("Invalid command id: " + it.key());
        }
        json args = j["query"][it.key()];
        if(cfg->getCmdType() == CommandConfig::Type::Feedback) {
            list.push_front(new SensorCommand(*(FeedbackConfig*)(cfg), args));
        } else if(cfg->getCmdType() == CommandConfig::Type::Arm) {
            list.push_front(new ArmQueryCommand(*(ArmConfig*)(cfg), args));
        } else {
            throw std::invalid_argument("Invalid feedback id " + it.key());
        }
    }
    ITER_VERB("saved") {
        CommandConfig *cfg = cfgManager.getConfigMap()[it.key()];
        CommandConfig::Type type = cfg->getCmdType();
        // If it's a saved command then add all the pieces to the list
        if(type == CommandConfig::Type::SavedCommand) {
            SavedCommandConfig *svCfg = (SavedCommandConfig *)cfg;
            auto cmdCfgs = svCfg->getCmds();
            for(auto cfgPair : *cmdCfgs) {
                size++; // count all the internal commands
                auto cfg = cfgManager.getConfigMap()[cfgPair.first];
                addMoveToList(cfgPair.second, list, cfg);
            }
            delete cmdCfgs;
        } else {
            throw std::invalid_argument("Invalid saved command id " + it.key());
        }
    }
    if(size == 0) {
        throw std::invalid_argument("No valid commands to fufill");
    }
    return std::pair<uint, std::forward_list<Command*>>(size, list);
}

void 
CommandListener::addMoveToList(json &args, 
                               std::forward_list<Command*> &list, 
                               CommandConfig *cfg) {
    CommandConfig::Type type = cfg->getCmdType();
    switch(type) {
        case CommandConfig::Type::Package:
            {
                std::string cmdType = args["type"];
                PackageConfig *pkgCfg = (PackageConfig *)cfg;
                if(cmdType.compare("relax") == 0) {
                    list.push_front(new PackageRelaxCommand(*pkgCfg, args));
                } else {
                    list.push_front(new PackageCommand(*pkgCfg, args));
                }
            }
            break;
        case CommandConfig::Type::Eye:
            {
                EyeConfig *eCfg = (EyeConfig *)cfg;
                list.push_front(new EyeCommand(*eCfg, args));
            }
            break;
        case CommandConfig::Type::Arm:
            {
                std::string cmdType = args["type"];
                ArmConfig *aCfg = (ArmConfig *)cfg;
                if(cmdType.compare("relax") == 0) {
                    list.push_front(new ArmRelaxCommand(*aCfg, args));
                } else {
                    list.push_front(new ArmCommand(*aCfg, args));
                }
            }
            break;
        default:
            throw std::invalid_argument("Invalid move id");
    }
}