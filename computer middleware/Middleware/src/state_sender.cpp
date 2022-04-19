#include <iostream>
#include "../include/state_sender.hpp"

#define UDP
#define PACKET_SIZE 1024

void StateSender::send(std::string mcuId,
std::forward_list<std::pair<int, uint8_t*>> cmds) {
    // TODO: Aggregate the commands into a smaller set of commands

    // Send the command(s) over TCP
    auto sock = mcuSockets[mcuId];
    for(auto cmdTuple : cmds) {
        int length = cmdTuple.first;
        #ifdef PRINT_SEND
        std::cout << "SENDING: ";
        for(int i = 0; i < length; i++) {
            std::cout << "0x" << std::hex << (int)(cmdTuple.second[i]) << " ";
        }
        std::cout << std::dec << std::endl;
        #endif
        #ifdef LOG_FILE
        log.writeByteLine('S', cmdTuple.second, length);
        #endif
        ::send(sock, (void *)cmdTuple.second, length, 0);
        delete[] cmdTuple.second;
    }
}

// returns a json parsed response, address and package
std::tuple<json,uint16_t,uint8_t>
StateSender::parseRecv(uint8_t *buf) {
    uint16_t addr = 0;
    uint8_t pkg = 0;
    json resJson;
    // Parse recieved bytes
    if(buf[0] == OP_RESPOND) {
        uint16_t cmd = 0;
        uint8_t len = 0;
        addr |= buf[1] << 8;
        addr |= buf[2];
        pkg |= buf[3];
        cmd |= buf[4] << 8;
        cmd |= buf[5];
        len |= buf[6];
        // Output is always a number
        if(len == 4) {
            int32_t out = 0;
            out |= buf[7] << 24;
            out |= buf[8] << 16;
            out |= buf[9] << 8;
            out |= buf[10];
            float fOut;
            memcpy(&fOut, &out, 4);
            resJson = {
                {"response", fOut}
            };
        } else {
            resJson = {
                {"status", "success"}
            };
        }
    } else if(buf[0] == OP_ERROR) {
        std::cout << "Error: issue sending command" << std::endl;
        addr |= buf[1] << 8;
        addr |= buf[2];
        pkg |= buf[3];
        // Parse error codes
        uint8_t errCode = buf[4];
        resJson = {
            {"error", parseErrCode(errCode)}
        };
    } else {
        throw std::logic_error("unknown response recieved");
    }
    return std::tuple<json,uint16_t,uint8_t>(resJson,addr,pkg);
}

std::map<std::string, std::forward_list<json>> 
StateSender::pollRecieve(std::string mcuId, 
                         std::map<uint32_t,std::string> &addrToId) {
    std::map<std::string, std::forward_list<json>> responses;
    auto sock = mcuSockets[mcuId];
    ssize_t lenRecv = 0;
    while(lenRecv != -1) {
        auto *buf = new uint8_t[PACKET_SIZE];
        lenRecv = ::recv(sock, buf, PACKET_SIZE, MSG_DONTWAIT);
        if(lenRecv != -1) {
            #ifdef PRINT_RECV
            std::cout << "RECV: ";
            for(int i = 0; i < lenRecv; i++) {
                std::cout << "0x" << std::hex << (int)(buf[i]) << " ";
            }
            std::cout << std::endl;
            #endif
            #ifdef LOG_FILE
            log.writeByteLine('R', buf, lenRecv);
            #endif
            uint16_t addr;
            uint8_t pkg;
            json resJson;
            try {
                std::tie(resJson, addr, pkg) = parseRecv(buf);
            } catch(std::logic_error &e) {
                std::cout << "Error: Unknown response recieved" << std::endl;
                for(int i = 0; i < lenRecv; i++) {
                    std::cout << "0x" << std::hex << buf[i] << " " << std::dec;
                }
                std::cout << std::endl;
                // TODO: Stop on unknown response? Could mean the MCU was reset?
                continue;
            }
            // Use the given map to back reference commands
            uint32_t fullAddr = 0;
            fullAddr |= addr << 8;
            fullAddr |= pkg;
            auto id = addrToId[fullAddr];
            if(id.compare("") != 0) {
                #ifdef PRINT_DEBUG
                std::cout << "Put into ID " << id << std::endl;
                #endif
                // add full address to the response
                resJson["fullAddr"] = fullAddr;
                responses[id].push_front(resJson);
            }
        }
        delete[] buf;
    }
    return responses;
}

std::string StateSender::parseErrCode(uint8_t code) {
    switch(code) {
        case(ERR_UNKNOWN_OPCODE):
            return "Unknown opcode";
        case(ERR_UNKNOWN_MODULE):
            return "Unknown module";
        case(ERR_UKNOWN_ADDRESS):
            return "Unknown address";
        case(ERR_UNKNOWN_PACKAGE):
            return "Unknown package";
        case(ERR_UNKNOWN_COMMAND):
            return "Unknown command";
        case(ERR_PARAMS_OUT_OF_RANGE):
            return "Parameters out of range";
        case(ERR_INVALID_ORDER):
            return "Invalid order";
        case(ERR_NOT_IMPLEMENTED):
            return "Not implemented";
    }
    return "Unknown error";
}

int
StateSender::connectToMcu(std::string mcuId, 
    std::pair<std::string,int> strAddr) {
    // Create socket
    #ifdef UDP
    auto newSock = socket(AF_INET, SOCK_DGRAM, 0);
    #elif TCP
    auto newSock = socket(AF_INET, SOCK_STREAM, 0);
    #else
    #error Define TCP or UDP
    #endif
    parseSockErrno(newSock);
    struct in_addr parsedAddr;
    if(inet_pton(AF_INET, strAddr.first.c_str(), &parsedAddr) != 1) {
        std::cout << "Invalid address for " << mcuId << std::endl;
        throw std::exception();
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(strAddr.second);
    addr.sin_addr = parsedAddr;
    auto out = connect(newSock, (struct sockaddr *)&addr, 
        sizeof(struct sockaddr_in));
    parseSockErrno(out);
    return newSock;
}

void StateSender::initMcu(std::string mcuId, 
std::forward_list<CommandConfig*> devices) {
    std::cout << "Connecting to microcontroller..." << std::endl;
    int newSock = connectToMcu(mcuId, mcuAddrs[mcuId]);
    mcuSockets[mcuId] = newSock;
    // Send software reset command
    std::cout << "Sending software reset to microcontroller" << std::endl;
    ::send(newSock, RESET, RESET_LEN, 0);
    close(newSock);
    // wait a second
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Trying to reconnect to microcontroller..." << std::endl;
    newSock = connectToMcu(mcuId, mcuAddrs[mcuId]);
    std::cout << "Reconnected, sending SYN command" << std::endl;
    // Send SYN command
    ::send(newSock, SYN, SYN_LEN, 0);
    confAck(newSock);
    std::cout << "Assign addresses and package ids to devices:" << std::endl;
    for(auto device : devices) {
        // Create map of addresses of devices to the string ids
        uint16_t addr = device->getAddr();
        std::string pkgAddr = std::to_string(device->getPkgAddr());
        // 65535 is the address of a device that doesn't use an address 
        if(addr != 65535) {
            std::cout << addr << " A-> " << device->id << std::endl;
            std::cout << pkgAddr << " P-> " << device->id << std::endl;
        }
        // Send initialization commands
        auto initCmd = device->getInitCmd();
        if(initCmd.first > 0) {
            ::send(newSock, (void *)initCmd.second, initCmd.first, 0);
            confAck(newSock);
            delete[] initCmd.second;
        }
    }
    ::send(newSock, FIN, FIN_LEN, 0);
    confAck(newSock);
    std::cout << "Configuration complete" << std::endl;
    // Send startup commands
    for(auto device : devices) {
        auto cmds = device->getSetupCommands();
        if(cmds == NULL) {
            continue;
        }
        for(auto cmd : *cmds) {
            auto timer = std::chrono::steady_clock::now();
            auto loopTime = std::chrono::microseconds(0);
            while(cmd->step(loopTime)) {
                // Send the command, ignore the response
                auto data = cmd->interpret();
                ::send(newSock, data.second, data.first, 0);
                delete[] data.second;
                auto buf = new uint8_t[PACKET_SIZE];
                ::recv(newSock, buf, PACKET_SIZE, 0);
                delete[] buf;
                loopTime = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - timer);
                timer = std::chrono::steady_clock::now();
            }
            delete cmd;
        }
        delete cmds;
    }
    // Initialize package positions
    for(auto device : devices) {
        if(device->getCmdType() == CommandConfig::Type::Package) {
            auto feedback = ((PackageConfig*)device)->feedback;
            if(!feedback) {
                std::cout << "feedback not present for package: " << device->id <<
                    " not querying initial motor position" << std::endl;
                continue;
            }
            bool a = false;
            std::pair<int, uint8_t*> data = interpSensorCommand(feedback->getAddr(), 
                feedback->getPkgAddr(), a);
            ::send(newSock, data.second, data.first, 0);
            auto buf = new uint8_t[PACKET_SIZE];
            ::recv(newSock, buf, PACKET_SIZE, 0);
            json resJson;
            std::tie(resJson, std::ignore, std::ignore) = parseRecv(buf);
            if(resJson["response"].is_number()) {
                ((PackageConfig*)device)->actuator.setCurPos(resJson["response"]);
                std::cout << "initial position of package: " << device->id
                    << " is: " << resJson["response"] << std::endl;
            } else {
                std::cout << "tried to query package: " + device->id + 
                    "'s sensor, but didn't get a usable response" << std::endl;
                std::cout << "got: " << resJson << std::endl;
            }
            delete[] buf;
            delete[] data.second;
        }
    }
}

void StateSender::confAck(int sock) {
    auto *buf = new uint8_t[PACKET_SIZE];
    auto ret = ::recv(sock, buf, PACKET_SIZE, 0);
    if(ret == -1) {
        std::cout << "No response from microcontroller: "
            << strerror(errno) << std::endl;
        delete[] buf;
        raise(SIGINT);
    }
    if(buf[0] == OP_ERROR) {
        delete[] buf;
        std::cout << "Error with initialization: " << parseErrCode(buf[4]) << std::endl;
        raise(SIGINT);
    } else if(buf[0] != OP_ACK) {
        delete[] buf;
        std::cout << "Unknown response from microcontroller" << std::endl;
        raise(SIGINT);
    }
    delete[] buf;
}

void StateSender::parseSockErrno(int code) {
    if(code == -1) {
        // error creating socket
        std::cout << "Could not create socket: " <<
            strerror(errno) << std::endl;
        raise(SIGINT);
    }
}

void StateSender::closeSockets() {
    for(auto sockPair : mcuSockets) {
        std::cout << "Closing connection to: " << sockPair.first << std::endl;
        // Send FIN command
        ::send(sockPair.second, FIN, FIN_LEN, 0);
        close(sockPair.second);   
    }
}