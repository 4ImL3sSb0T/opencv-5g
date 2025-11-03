#ifndef __CONFIG_H
#define __CONFIG_H

#include "json.hpp"
#include <filesystem>
#include <fstream>

namespace Config {
    nlohmann::json j_config;

    bool load_config(const std::string& path) {
        std::ifstream config_file(path);
        if (!config_file.is_open()) {
            return false;
        }
        config_file >> j_config;
        return true;
    }

    nlohmann::json get_config() {
        return j_config;
    }
}

#endif // !__CONFIG_H
