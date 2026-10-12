#pragma once

#include "calibration_base.h"

class RuiwoActuatorBase;

namespace leju {
namespace hw {

class HumanoidCali : public CalibrationBase {
public:
    HumanoidCali(const std::string& config_dir, CaliMode mode, CaliType type);
    virtual ~HumanoidCali();

    virtual bool init() override;
    virtual bool cali() override;

private:
    /**
     * @brief 卡工装校准
     */
    bool toolingCali();

    /**
     * @brief CANFD/RUIWO 电机工装标定 (交互式)
     */
    bool toolingCaliCanFd(RuiwoActuatorBase* ruiwo);

    /**
     * @brief 利用关节限位校准
     */
    bool jointLimitedCali();

    /**
     * @brief 手动从当前位置 +- 圈数进行校准
     */
    bool manualCali();
};

} // namespace leju
} // namespace hw