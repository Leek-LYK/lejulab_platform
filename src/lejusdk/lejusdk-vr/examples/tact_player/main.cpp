#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <lejusdk-vr/lejusdk_vr.h>

#include "bezier_interpolator.h"
#include "tact_parser.h"
#include "trajectory_publisher.h"

namespace {

std::atomic<bool> g_running{true};

void OnSignal(int signum) {
  std::cerr << "received signal " << signum << ", stopping" << std::endl;
  g_running.store(false);
}

void PrintUsage(const char* bin) {
  std::cout << "Usage:\n"
            << "  " << bin
            << " --tact <file> [--robot kuavo|roban]\n";
}

bool ReadArgValue(const std::vector<std::string>& args,
                  std::size_t i,
                  std::string* value,
                  std::string* err) {
  if (i + 1 >= args.size()) {
    *err = "missing value for " + args[i];
    return false;
  }
  *value = args[i + 1];
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  signal(SIGINT, OnSignal);
  signal(SIGTERM, OnSignal);

  std::vector<std::string> args(argv + 1, argv + argc);
  std::string tact_file;
  std::string robot = "kuavo";

  for (std::size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--tact") {
      std::string value;
      std::string err;
      if (!ReadArgValue(args, i, &value, &err)) {
        std::cerr << err << std::endl;
        PrintUsage(argv[0]);
        return 1;
      }
      tact_file = value;
      ++i;
    } else if (args[i] == "--robot") {
      std::string value;
      std::string err;
      if (!ReadArgValue(args, i, &value, &err)) {
        std::cerr << err << std::endl;
        PrintUsage(argv[0]);
        return 1;
      }
      robot = value;
      ++i;
    } else if (args[i] == "--help" || args[i] == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "unknown argument: " << args[i] << std::endl;
      PrintUsage(argv[0]);
      return 1;
    }
  }

  if (tact_file.empty()) {
    std::cerr << "--tact is required" << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }

  std::size_t arm_dof = 14;
  int waist_index = -1;  // -1 means no waist
  std::unique_ptr<leju::vr::VRBaseAPI> api;
  if (robot == "kuavo") {
    arm_dof = 14;
    waist_index = -1;  // kuavo 4.x has no waist, kuavo 5.x has waist at index 28
    api = std::make_unique<leju::vr::KuavoVRAPI>();
  } else if (robot == "roban") {
    arm_dof = 8;
    waist_index = 22;  // roban has waist at index 22
    api = std::make_unique<leju::vr::RobanVRAPI>();
  } else {
    std::cerr << "--robot only supports kuavo|roban" << std::endl;
    return 1;
  }

  if (!api->initialize()) {
    std::cerr << "failed to initialize VR API" << std::endl;
    return 1;
  }

  leju::vr::tact_player::TactAction action;
  std::string parse_error;
  leju::vr::tact_player::ParseOptions parse_options;
  parse_options.arm_dof = arm_dof;
  parse_options.waist_index = waist_index;
  if (!leju::vr::tact_player::LoadTactFile(tact_file, parse_options, &action, &parse_error)) {
    std::cerr << "failed to parse tact: " << parse_error << std::endl;
    api->shutdown();
    return 1;
  }

  leju::vr::tact_player::BezierInterpolator interpolator;
  leju::vr::tact_player::InterpolateOptions interpolate_options;
  interpolate_options.arm_dof = arm_dof;
  interpolate_options.speed_scale = 1.0;
  interpolate_options.enable_waist = (waist_index >= 0);
  if (!interpolator.Build(action, interpolate_options, &parse_error)) {
    std::cerr << "failed to build interpolation: " << parse_error << std::endl;
    api->shutdown();
    return 1;
  }

  std::cout << "set arm mode -> external" << std::endl;
  if (!api->setArmMode(leju::vr::ControlMode::kExternal)) {
    std::cerr << "set arm mode failed" << std::endl;
    api->shutdown();
    return 1;
  }

  // Set waist mode if waist is enabled
  const bool has_waist = interpolator.HasWaist();
  if (has_waist) {
    std::cout << "set waist mode -> external" << std::endl;
    if (!api->setWaistMode(leju::vr::ControlMode::kExternal)) {
      std::cerr << "set waist mode failed" << std::endl;
      api->shutdown();
      return 1;
    }
  }

  leju::vr::tact_player::TrajectoryPublisher publisher(api.get(), &interpolator, &g_running);
  publisher.Run();

  std::cout << "set arm mode -> auto" << std::endl;
  api->setArmMode(leju::vr::ControlMode::kAuto);

  if (has_waist) {
    std::cout << "set waist mode -> auto" << std::endl;
    api->setWaistMode(leju::vr::ControlMode::kAuto);
  }

  api->shutdown();
  return 0;
}
