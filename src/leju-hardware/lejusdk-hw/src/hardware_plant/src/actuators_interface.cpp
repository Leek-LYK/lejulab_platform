#include "actuators_interface.h"

#ifdef ENABLE_ECMASTER

#include <filesystem>
#include <iostream>
#include <cstdlib>
#include "kuavo_common/common/utils.h"
#include "kuavo_common/common/json_config_reader.hpp"

pthread_t ecmaster_thread;

// 读取环境变量中的 ROBOT_VERSION，例如 "53" / "54"
static std::string getRobotVersionFromEnv()
{
    const char* robot_version = std::getenv("ROBOT_VERSION");
    if (robot_version != nullptr)
    {
        return std::string(robot_version);
    }
    return "";
}

std::string load_ec_master_license()
{
    std::string ec_master_path = getKuavoEcMasterLicensePath();
    std::string content = "";
    std::ifstream file(ec_master_path);
#define VALID_EC_MASTER_LICENSE_STR_LEN 26
    if (!file)
    {
        std::cerr << "EC_master License: File " << ec_master_path << " does not exist." << std::endl;
    }
    else
    {
        std::string line;
        int lineCount = 0; // Add a counter for lines
        while (std::getline(file, line))
        {
            content += line;
            lineCount++;
        }
        if (lineCount > 1)
        {
            content = "";
            std::cerr << "EC_master License: File Format is wrong, have more than one lines string: " << content << std::endl;
        }
        else
        {
            content.erase(std::remove_if(content.begin(), content.end(), ::isspace), content.end());
            int contentLen = content.length();
            if (contentLen != VALID_EC_MASTER_LICENSE_STR_LEN)
            {
                std::cerr << "EC_master License len is invalid : the license str len is: " << contentLen << " The valid length is: " << VALID_EC_MASTER_LICENSE_STR_LEN << std::endl;
                content = "";
            }
            std::cout << "Extract ec_master license is: " << content << std::endl;
        }
    }
    return content;
}

void *setup_Ec_Master(void *arg)
{
    sched_thread(80);
    EcActuatorParams *params = (EcActuatorParams *)arg;
    int countECMasters = params->num_actuators;
    std::string ec_type = params->ec_type;
    std::string config_dir = params->config_dir;  // lib/EC_Master/config/
    std::string config_path;

    // 根据 ROBOT_VERSION 判断是否为 53/54 版本的 kuavo
    std::string robot_version = getRobotVersionFromEnv();
    bool is_robot_version_53 = (robot_version == "53");
    bool is_robot_version_54 = (robot_version == "54");

    // 与 kuavo-ros-control 保持一致：V53/V54 走专用 XML，其它版本走通用规则
    if (is_robot_version_53)
    {
        config_path = config_dir + "Kuavo5_T26_kpkd_ntc.xml";
    }
    else if (is_robot_version_54)
    {
        config_path = config_dir + "Kuavo5_T25_V54.xml";
    }
    else if (ec_type == "elmo")
    {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "elmo_%d_c500.xml", countECMasters);
        config_path = config_dir + buffer;  //"_busshift.xml"
    }
    else if (ec_type == "youda1" || ec_type == "youda")
    {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "yd_%d_c500.xml", countECMasters);
        config_path = config_dir + buffer;
    }
    else if (ec_type == "youda3")
    {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "yd300_%d_c500.xml", countECMasters);
        config_path = config_dir + buffer;
    }
    else if (ec_type == "leju")
    {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "selfd_%d_c500.xml", countECMasters);
        config_path = config_dir + buffer;
    }
    else if(ec_type == "lunbi")
    {
        config_path = config_dir + "lb_6_c500.xml";
    }
    else
    {
        std::cerr << "ecmaster_type:"<< ec_type << " is not supported! Please check the config file." << std::endl;
        exit(1);
    }

    std::cout << "Ec_Master config_path: " << config_path << std::endl;

    std::string ec_master_license_str = load_ec_master_license();


    // 获取主机名
    char hostname[256] = {0};
    if (gethostname(hostname, sizeof(hostname)) != 0) {
        std::cerr << "Warning: Failed to get hostname, using default network port configuration." << std::endl;
        strcpy(hostname, "unknown");
    }
    std::string hostname_str(hostname);
    std::cout << "Detected hostname: " << hostname_str << std::endl;

    // 根据主机名和ROBOT_VERSION选择网口号
    std::string network_port_arg;
    if (params->robot_module == "LUNBI") {
        network_port_arg = "-i8254x 1 1";
    } else if (hostname_str == "AAEON") {
        if (is_robot_version_53) {
            std::cout << "Robot version 53 detected on AAEON, using single network port configuration." << std::endl;
            network_port_arg = "-i8254x 2 1";
        } else {
            network_port_arg = "-i8254x 2 1 -i8254x 3 1";
        }
    } else {
        network_port_arg = "-i8254x 1 1 -i8254x 2 1";
    }
    std::cout << "Selected network port argument: " << network_port_arg << std::endl;

    char *command[] = {
        (char *)"main",
        (char *)network_port_arg.c_str(),
        (char *)"-a",
        (char *)"7",
        (char *)"-v",
        (char *)"2",
        (char *)"-f",
        (char *)config_path.c_str(),
        (char *)"-auxclk",
        (char *)(params->robot_module == "ROBAN2" ? "1000" : "500"),
        (char *)"-dcmmode",
        (ec_type == "elmo")?(char *)"busshift":(char *)"mastershift",
        (char *)"-t",
        (char *)"0",
        (char *)"-lic",
        (char *)ec_master_license_str.c_str(),
    }; // keep this argument at last of command, for when ec_master license is not provided

    int ec_master_argc;

    ec_master_argc = sizeof(command) / sizeof(command[0]);

    if (ec_master_license_str.length() <= 0)
    {
        // ec_master_license is invalid, remove from the argv of ec_master_init;
        ec_master_argc -= 2;
    }

    Ec_Master_init(ec_master_argc, command);
    std::cout << "Ecmaster exited\n";
    std::raise(SIGINT);
    return NULL;
}

int ECMaster_init(EcActuatorParams params)
{
    int ret = 0;
    ret = pthread_create(&ecmaster_thread, NULL, setup_Ec_Master, (void *)&params);
    if (ret != 0)
    {
        printf("Create lcm_thread failed! %s\n", strerror(ret));
        return -1;
    }

    while (1)
    {
        if (isMotorEnable())
        {
            break;
        }
        OsSleep(1000);
    }
    std::cout << "ECMaster_init DONE !\n";
    return 0;
}
void EM_deInit(void)
{
    EcMasterStop();
}
int8_t actuatorsInterfaceSetup(const char *type, ActuatorsInterface_t *interfacePtr)
{
    if (strcmp(type, "real") == 0)
    {
        interfacePtr->init = ECMaster_init;
        interfacePtr->deInit = EM_deInit;
        interfacePtr->setJointPosition = motorSetPosition;
        interfacePtr->setJointVelocity = motorSetVelocity;
        interfacePtr->setJointTorque = motorSetTorqueWithFeedback;
        interfacePtr->setEncoderRange = setEcEncoderRange;

        interfacePtr->getJointData = motorGetData;
        interfacePtr->addIgnore = disableMotor;
        interfacePtr->setRobotMoudle = setRobotMoudle;

        interfacePtr->setJointKp = motorSetkp;
        interfacePtr->setJointKd = motorSetkd;

        interfacePtr->readJointKp = motorReadKp;
        interfacePtr->readJointKd = motorReadKd;

        interfacePtr->writeJointKp = motorWriteKp;
        interfacePtr->writeJointKd = motorWriteKd;
        printf("Info: actuatorsInterfaceSetup success!\n");
        return 0;
    }
    printf("Error: actuatorsInterfaceSetup failed!\n");
    return -1;
}

#else // !ENABLE_ECMASTER

#include <cstdio>
#include <cstring>

int ECMaster_init(EcActuatorParams params)
{
    printf("Warning: EC-Master not available on this platform\n");
    return -1;
}

int8_t actuatorsInterfaceSetup(const char *type, ActuatorsInterface_t *interfacePtr)
{
    printf("Warning: EC-Master actuators interface not available on this platform\n");
    return -1;
}

#endif // ENABLE_ECMASTER
