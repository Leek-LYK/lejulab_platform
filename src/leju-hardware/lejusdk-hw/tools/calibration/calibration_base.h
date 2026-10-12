#pragma once

#include <memory>
#include "hardware_plant.h"

namespace leju {
namespace hw {

/**
 * @brief Calibration mode enumeration
 *
 * Defines different calibration operational modes for the calibration process.
 */
enum class CaliMode : int32_t {
    None = 0,
    Tooling = 1,        ///< Automated tool-based calibration
    Manual = 2,         ///< Manual calibration with user interaction
    JointLimited = 3    ///< Calibration with joint movement constraints
};

/**
 * @brief Calibration type enumeration
 *
 * Defines the scope of calibration by specifying which body parts to calibrate.
 */
enum class CaliType : int32_t {
    None = 0,
    FullBody = 1,  ///< Complete body calibration (alias: All-Joint)
    UpperBody = 2, ///< Upper body only (alias: ArmHead-Only)
    LowerBody = 3, ///< Lower body only (alias: Leg-Only, includes waist if present)
    Single = 4     ///< Single motor calibration: only the operator-selected motor is zeroed,
                   ///< all other motors are loaded as-is from the existing zero file (no bulk reset)
};



/**
 * @brief Base class for calibration implementations
 *
 * This abstract base class provides the common interface and functionality for
 * different calibration implementations. It defines the calibration modes,
 * types, and the core calibration method that derived classes must implement.
 *
 * Derived classes should implement the pure virtual cali() method to provide
 * specific calibration logic for different hardware configurations.
 */
class CalibrationBase {
public:
    CalibrationBase(const std::string& config_dir, CaliMode mode, CaliType type)
    : config_dir_(config_dir), mode_(mode), type_(type), hardware_plant_(nullptr) {
  }

  virtual ~CalibrationBase() = default;

    /**
     * @brief Initialize calibration system
     *
     * This method should be called before cali() to initialize the calibration
     * system, set up hardware connections, and prepare for the calibration process.
     *
     * @return true if initialization was successful, false otherwise
     */
    virtual bool init() = 0;

    /**
     * @brief Pure virtual calibration method
     *
     * This method must be implemented by derived classes to provide the actual
     * calibration logic. The implementation should perform the calibration
     * procedure based on the mode_ and type_ settings specified during construction.
     *
     * @return true if calibration was successful, false otherwise
     */
    virtual bool cali() = 0;

protected:
    CaliMode mode_; ///< Current calibration mode
    CaliType type_; ///< Current calibration type
    std::string config_dir_; ///< Configuration directory path

    HighlyDynamic::HardwarePlant* hardware_plant_ = nullptr; ///< Hardware plant instance for calibration operations
    std::atomic<bool> initialized_{false};
};

} // namespace leju
} // namespace hw    