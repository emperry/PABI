#include <signal.h>
#include "../include/config_manager.hpp"

struct MemoryStruct {
  char *memory;
  size_t size;
};

// https://curl.haxx.se/libcurl/c/getinmemory.html
static size_t writeMem(void *contents, size_t size, 
    size_t nmemb, void *userp) {
      size_t realsize = size * nmemb;
    struct MemoryStruct *mem = (struct MemoryStruct *)userp;
    
    char *ptr = (char *)realloc(mem->memory, mem->size + realsize + 1);
    if(ptr == NULL) {
        /* out of memory! */ 
        printf("not enough memory (realloc returned NULL)\n");
        return 0;
    }
    
    mem->memory = ptr;
    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;
    
    return realsize;
}

// Uses the given database to retrieve and parse
// the configuration for the MSARF
void ConfigurationManager::generateConfigs() {
    struct MemoryStruct chunk;
    chunk.memory = (char *)malloc(1);
    chunk.size = 0;
    auto curlHandle = curl_easy_init();
    // The view init_order gives all of the msarf_config
    // documents in the correct order of initialization so
    // dependencies are initialized before higher level objects
    std::string fullUrl = this->url + "/" + this->database + 
        "/_design/views/_view/init_order?include_docs=true";
    std::cout << "Using database at " << fullUrl << std::endl;
    curl_easy_setopt(curlHandle, CURLOPT_URL, fullUrl.c_str());
    curl_easy_setopt(curlHandle, CURLOPT_WRITEFUNCTION, writeMem);
    curl_easy_setopt(curlHandle, CURLOPT_WRITEDATA, (void *)&chunk);
    auto res = curl_easy_perform(curlHandle);
    if(res != CURLE_OK) {
        std::cout << "Failed to get configs " << res << std::endl;
        raise(SIGINT);
    } else {
        std::cout << "Recieved " << chunk.size << " bytes from database" << std::endl;
        // Parse JSON
        json data = json::parse(chunk.memory);
        // std::cout << data << std::endl;
        configMap = genConfigMap(data);
        std::cout << "Parsed database config" << std::endl;
    }
    curl_easy_cleanup(curlHandle);
    free(chunk.memory);
    curl_global_cleanup();
}

std::map<std::string, CommandConfig*>
ConfigurationManager::genConfigMap(json data) {
    std::map<std::string, CommandConfig*> map;
    for(auto config : data["rows"]) {
        auto doc = config["doc"];
        std::string id = doc["_id"];
        // initially set the map at this document to null
        map[id] = NULL;
        // generate command configs using the data in each document
        // skip the document if it isn't one of the valid types
        if(!doc["type"].is_string()) {
            std::cout << "Invalid document with id " + id << std::endl;
            continue;
        }
        
        std::string type = doc["type"];
        if(type.compare("actuator") == 0) {
            std::string actuatorType = doc["actuator_type"];
            float minPos = 0;
            float maxPos = 6.28; // 2PI
            if(doc["min_pos"].is_number()) {
                minPos = doc["min_pos"];
            }
            if(doc["max_pos"].is_number()) {
                maxPos = doc["max_pos"];
            }
            if(actuatorType.compare("dcmotor") == 0) {
                uint8_t pwmPin = doc["pwm_pin"];
                map[id] = new BrushedMotorConfig(id, minPos, maxPos, 
                    pwmPin);
            } else if(actuatorType.compare("pwmservo") == 0) {
                uint8_t pwmPin = doc["pwm_pin"];
                uint8_t maxRot = doc["max_rot"];
                map[id] = new PWMServoConfig(id, minPos, maxPos,
                    pwmPin, maxRot);
            } else if(actuatorType.compare("xyzsmartservo") == 0) {
                uint8_t uartAddr = doc["uart_addr"];
                map[id] = new XYZSmartServoConfig(id, minPos, maxPos,
                    uartAddr);
            } else if(actuatorType.compare("dynamixelservo") == 0) {
                uint8_t uartAddr = doc["uart_addr"];
                map[id] = new DynamixelServoConfig(id, minPos, maxPos, uartAddr);
            } else if(actuatorType.compare("steppermotor") == 0) {
                uint8_t stepPin = doc["step_pin"];
                uint8_t dirPin = doc["dir_pin"];
                uint8_t enPin = doc["en_pin"];
                map[id] = new StepperMotorConfig(id, minPos, maxPos,
                    stepPin, dirPin, enPin);
            } else if(actuatorType.compare("binary") == 0) {
                uint8_t pin = doc["pin"];
                map[id] = new BinaryOutputConfig(id, pin);
            } else if(actuatorType.compare("brushlessmotor") == 0) {
                uint8_t pin = doc["pin"];
                map[id] = new BrushlessMotorConfig(id, minPos, maxPos, pin);
            } else {
                throw std::logic_error("Invalid actuator type " + actuatorType);
            }
            float velocity = 0.6;
            if(doc["velocity"].is_number()) {
                float v = doc["velocity"];
                velocity = std::abs(v);
            }
            ((ActuatorConfig*)map[id])->velocity = velocity;
        } else if(type.compare("feedback") == 0) {
            std::string type = doc["sensor_type"];
            if(type.compare("pot") == 0) {
                uint8_t pin = doc["pin"];
                map[id] = new PotentiometerConfig(id, pin);
            } else if(type.compare("encoder") == 0) {
                uint8_t pin = doc["pin"];
                uint16_t ticksPerRev = doc["ticks_per_rev"];
                map[id] = new EncoderConfig(id, pin, ticksPerRev);
            } else if(type.compare("binary") == 0) {
                uint8_t pin = doc["pin"];
                bool no = doc["normally_open"];
                map[id] = new BinaryInputConfig(id, pin, no);
            } else if(type.compare("xyzsmartsensor") == 0) {
                uint8_t uartAddr = doc["uart_addr"];
                map[id] = new XYZSmartSensorConfig(id, uartAddr);
            } else if(type.compare("spiencoder") == 0) {
                uint8_t chainSize = doc["chain_size"];
                uint8_t addr = doc["addr"];
                map[id] = new SPIEncoderConfig(id, chainSize, addr);
            } else if(type.compare("dynamixelsensor") == 0) {
                uint8_t uartAddr = doc["uart_addr"];
                map[id] = new DynamixelSensorConfig(id, uartAddr);
            } else {
                throw std::logic_error("Invalid sensor type " + type);
            }
        } else if(type.compare("saved_command") == 0) {
            map[id] = new SavedCommandConfig(id, doc);
        } else if(type.compare("package") == 0) {
            std::string actuatorId = doc["actuator_id"];
            PackageConfig *cfg = NULL;
            if(!map[actuatorId]) {
                throw std::runtime_error("Invalid actuator id for " + id);
            }
            if(doc["feedback_id"].is_string()) {
                std::string feedbackId = doc["feedback_id"];
                if(!map[feedbackId]) {
                    throw std::runtime_error("Invalid feedback id for " + id);
                }
                cfg = new PackageConfig(id, 
                    *((ActuatorConfig*)(map[actuatorId])), 
                    ((FeedbackConfig*)(map[feedbackId])), doc);
            } else {
                cfg = new PackageConfig(id,
                    *((ActuatorConfig*)(map[actuatorId])), doc);
            }
            if(doc["trans_mat"].is_string()) {
                cfg->setMatrixStr(doc["trans_mat"]);
            }
            if(cfg != NULL) {
                map[id] = cfg;
            }
        } else if(type.compare("arm") == 0) {
            auto jointPackages = std::vector<PackageConfig*>();
            for(std::string pkgId : doc["joint_package_ids"]) {
                jointPackages.push_back((PackageConfig*)(map[pkgId]));
            }
            map[id] = new ArmConfig(id, jointPackages);
        } else if(type.compare("eyes") == 0) {
            std::string id = doc["_id"];
            std::string lPT = doc["left_pixel_trans"];
            std::string rPT = doc["right_pixel_trans"];
            std::string lCT = doc["left_camera_trans"];
            std::string rCT = doc["right_camera_trans"];
            map[id] = new EyeConfig(id, lPT, rPT, lCT, rCT, NULL, NULL, 0, 0);
        } else {
            throw std::runtime_error("Invalid document found with type " + type);
        }

        if(map[id] != NULL) {
            // get constants to be supplied for each cfg and apply them
            float scaleFactor = 1;
            float zeroedPosition = 0;
            if(doc["scale_factor"].is_number()) {
                scaleFactor = doc["scale_factor"];
            }
            if(doc["zeroed_position"].is_number()) {
                zeroedPosition = doc["zeroed_position"];
            }
            map[id]->scaleFactor = scaleFactor;
            map[id]->zeroedPosition = zeroedPosition;
        }
    }
    return map;
}

std::map<std::string, CommandConfig*> &
ConfigurationManager::getConfigMap() {
    return configMap;
}

void ConfigurationManager::cleanUp() {
    for(auto cfg : this->configMap) {
        delete cfg.second;
    }
}
