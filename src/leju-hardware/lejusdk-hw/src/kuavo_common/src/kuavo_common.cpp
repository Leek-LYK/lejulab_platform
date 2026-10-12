#include <iostream>
#include <string>
#include <mutex>
#include <memory>
#include <filesystem>

#include "kuavo_common/kuavo_common.h"
#include "kuavo_common/common/json_config_reader.hpp"

namespace HighlyDynamic{

  std::shared_ptr<KuavoCommon> KuavoCommon::instance = nullptr;
  KuavoCommon &KuavoCommon::getInstance(const std::string& robot_name, const std::string& kuavo_assets_path)
  {
    if (!instance) {
        instance = std::shared_ptr<KuavoCommon>(new KuavoCommon(robot_name, kuavo_assets_path));
    }
    return *instance;
  }

  KuavoCommon *KuavoCommon::getInstancePtr(const std::string& robot_name, const std::string& kuavo_assets_path)
  {
    getInstance(robot_name, kuavo_assets_path);
    return instance.get();
  }

  KuavoCommon::~KuavoCommon()
  {
    if (robot_config_) {
      delete robot_config_;
      robot_config_ = nullptr;
    }
  }

  KuavoCommon::KuavoCommon(const std::string& robot_name, const std::string& kuavo_assets_path)
  {
    try {
        
        robot_config_ = new JSONConfigReader();
        std::cout << "Created JSONConfigReader" << std::endl;
        
        // x86 平台优先加载 kuavo_x86.json (EC-Master + USB-CAN 双驱动),
        // 不存在时回退到 kuavo.json (ARM 全 CAN FD/RUIWO 驱动).
#ifdef __aarch64__
        auto config_path = kuavo_assets_path + "/config/" + robot_name + "/kuavo.json";
#else
        auto config_path = kuavo_assets_path + "/config/" + robot_name + "/kuavo_x86.json";
        if (!std::filesystem::exists(config_path)) {
            config_path = kuavo_assets_path + "/config/" + robot_name + "/kuavo.json";
        }
#endif
        std::cout << "Will try to load config from: " << config_path << std::endl;

        if (!std::filesystem::exists(config_path)) {
            std::cerr << "Config file does not exist: " << config_path << std::endl;
            throw std::runtime_error("Config file not found");
        }
        
        robot_config_->load(config_path);
        std::cout << "Loaded config file" << std::endl;
        
        kuavo_settings_.kuavo_assets_path = kuavo_assets_path;
        kuavo_settings_.loadKuavoSettings(*robot_config_);
        std::cout << "Loaded kuavo settings" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in KuavoCommon constructor: " << e.what() << std::endl;
        throw;
    }
  }
}
