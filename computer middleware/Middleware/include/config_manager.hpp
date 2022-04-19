#include <string>
#include <map>
#include <curl/curl.h>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include "cmd_config.hpp"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#ifndef CONFIG_READER_H
#define CONFIG_READER_H

class ConfigurationManager {
    public:
        // Uses the given couchdb database url and database name
        // to create command configs. Exposes data structures to access
        // these configs.
        ConfigurationManager(std::string url, std::string database):
            url(url), database(database) {};
        void generateConfigs();
        std::map<std::string, CommandConfig*> &getConfigMap();
        void cleanUp();
    private:
        std::string url;
        std::string database;
        std::map<std::string, CommandConfig*> configMap;
        std::map<std::string, CommandConfig*> 
            genConfigMap(json data);
};

#endif