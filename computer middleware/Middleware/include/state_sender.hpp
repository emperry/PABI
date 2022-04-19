#include <map>
#include <string>
#include <string.h>
#include <cstdint>
#include <forward_list>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <signal.h>
#include "comms.hpp"
#include "cmd_config.hpp"
#include "command.hpp"
#include "build_consts.hpp"
#include "logging.hpp"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#ifndef STATE_SENDER_H
#define STATE_SENDER_H

class StateSender {
    public:
        StateSender(std::map<std::string, std::pair<std::string,int>> mcuAddrs)
        : mcuAddrs(mcuAddrs)
        #ifdef LOG_FILE
        , log("comms.log") { };
        #else
        {};
        #endif
        // Sends the given commands in a byte list (uint8_t*) 
        // with the given byte lengths
        void send(std::string mcuId, 
            std::forward_list<std::pair<int, uint8_t*>> cmds);
        // polls the given mcu's socket for any data to be recieved
        // stores this data in a map of addresses and the raw data
        std::map<std::string, std::forward_list<json>> 
            pollRecieve(std::string mcuId, 
                        std::map<uint32_t,std::string> &addrToId);
        void initMcu(std::string mcuId, 
            std::forward_list<CommandConfig*> devices);
        void closeSockets();
    private:
        // parses recieved packets
        std::tuple<json,uint16_t,uint8_t> parseRecv(uint8_t *buf);
        // Connects to an microcontroller given its ID and address tuple
        int connectToMcu(std::string mcuId, std::pair<std::string,int> strAddr);
        void parseSockErrno(int code);
        // Confirms ack packet, if it isn't an ack it raises an exception
        void confAck(int sock);
        std::string parseErrCode(uint8_t code);
        // pair is the IP address and the port
        std::map<std::string, std::pair<std::string,int>> mcuAddrs;
        std::map<std::string, int> mcuSockets;
        /*std::map<uint16_t, std::string> devAddrToId;
        std::map<uint16_t, std::string> pkgAddrToId;*/
        #ifdef LOG_FILE
        Logger log;
        #endif
};

#endif