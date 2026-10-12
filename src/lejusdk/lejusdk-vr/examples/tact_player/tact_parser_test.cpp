/**
 * @file tact_parser_test.cpp
 * @brief Independent test program for tact parsing and trajectory interpolation
 *
 * Usage: tact_parser_test <tact_file> [arm_dof]
 * Output: CSV files for keyframes and trajectory
 */

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "bezier_interpolator.h"
#include "tact_parser.h"

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

void PrintUsage(const char* bin) {
  std::cout << "Usage: " << bin << " <tact_file> [arm_dof]\n"
            << "\nOptions:\n"
            << "  arm_dof    Number of arm joints (default: 14 for kuavo, 8 for roban)\n"
            << "\nOutput:\n"
            << "  /tmp/<name>_keyframes.csv   - Parsed keyframe data\n"
            << "  /tmp/<name>_trajectory.csv  - Interpolated trajectory\n";
}

std::string GetBaseName(const std::string& path) {
  std::string base = path;
  auto pos = base.rfind('/');
  if (pos != std::string::npos) {
    base = base.substr(pos + 1);
  }
  pos = base.rfind('.');
  if (pos != std::string::npos) {
    base = base.substr(0, pos);
  }
  return base;
}

void PrintTactAction(const leju::vr::tact_player::TactAction& action, std::size_t arm_dof) {
  std::cout << "\n=== Parsed TactAction ===" << std::endl;
  std::cout << "robot_type: " << action.robot_type << std::endl;
  std::cout << "first_sec: " << action.first_sec << " s" << std::endl;
  std::cout << "finish_sec: " << action.finish_sec << " s" << std::endl;
  std::cout << "frames count: " << action.frames.size() << std::endl;

  for (std::size_t i = 0; i < action.frames.size(); ++i) {
    const auto& frame = action.frames[i];
    std::cout << "\n--- Frame " << i << " ---" << std::endl;
    std::cout << "keyframe: " << frame.keyframe << " (t=" << frame.keyframe * 0.01 << "s)" << std::endl;
    std::cout << "servos_deg[0.." << arm_dof - 1 << "]: [";
    for (std::size_t j = 0; j < arm_dof && j < frame.servos_deg.size(); ++j) {
      if (j > 0) std::cout << ", ";
      std::cout << std::fixed << std::setprecision(1) << frame.servos_deg[j];
    }
    std::cout << "]" << std::endl;
  }
}

void PrintTrajectory(const leju::vr::tact_player::BezierInterpolator& interpolator, double interval) {
  std::cout << "\n=== Interpolated Trajectory (sample interval=" << interval << "s) ===" << std::endl;
  std::cout << "Duration: " << interpolator.Duration() << " s" << std::endl;
  std::cout << "Arm DOF: " << interpolator.ArmDof() << std::endl;

  std::cout << std::setw(8) << "time(s)";
  for (std::size_t j = 0; j < interpolator.ArmDof(); ++j) {
    std::cout << std::setw(10) << ("j" + std::to_string(j));
  }
  std::cout << std::endl;

  for (double t = 0.0; t <= interpolator.Duration() + 0.001; t += interval) {
    const auto sample = interpolator.Evaluate(t);
    std::cout << std::fixed << std::setprecision(3) << std::setw(8) << t;
    for (std::size_t j = 0; j < sample.q.size(); ++j) {
      std::cout << std::fixed << std::setprecision(2) << std::setw(10) << (sample.q[j] * kRadToDeg);
    }
    std::cout << std::endl;
  }
}

bool SaveKeyframesToCsv(const leju::vr::tact_player::TactAction& action,
                        std::size_t arm_dof,
                        const std::string& csv_path) {
  std::ofstream ofs(csv_path);
  if (!ofs.is_open()) {
    std::cerr << "ERROR: cannot open " << csv_path << std::endl;
    return false;
  }

  ofs << "keyframe,time_s";
  for (std::size_t j = 0; j < arm_dof; ++j) {
    ofs << ",j" << j << "_deg,j" << j << "_left_cp_x,j" << j << "_left_cp_y,j" << j << "_right_cp_x,j" << j << "_right_cp_y";
  }
  ofs << "\n";

  for (const auto& frame : action.frames) {
    ofs << std::fixed << std::setprecision(0) << frame.keyframe
        << "," << std::setprecision(4) << (frame.keyframe * 0.01);
    for (std::size_t j = 0; j < arm_dof && j < frame.servos_deg.size(); ++j) {
      const auto& attr = frame.attributes[j];
      ofs << "," << std::fixed << std::setprecision(2) << frame.servos_deg[j]
          << "," << attr.cp[0][0] << "," << attr.cp[0][1]
          << "," << attr.cp[1][0] << "," << attr.cp[1][1];
    }
    ofs << "\n";
  }

  ofs.close();
  std::cout << "Keyframes saved to: " << csv_path << std::endl;
  return true;
}

bool SaveTrajectoryToCsv(const leju::vr::tact_player::BezierInterpolator& interpolator,
                         const std::string& csv_path,
                         double sample_interval) {
  std::ofstream ofs(csv_path);
  if (!ofs.is_open()) {
    std::cerr << "ERROR: cannot open " << csv_path << std::endl;
    return false;
  }

  ofs << "time_s";
  for (std::size_t j = 0; j < interpolator.ArmDof(); ++j) {
    ofs << ",j" << j << "_pos_deg,j" << j << "_vel_deg_s,j" << j << "_acc_deg_s2";
  }
  ofs << "\n";

  for (double t = 0.0; t <= interpolator.Duration() + 0.0001; t += sample_interval) {
    const auto sample = interpolator.Evaluate(t);
    ofs << std::fixed << std::setprecision(4) << t;
    for (std::size_t j = 0; j < sample.q.size(); ++j) {
      ofs << "," << std::fixed << std::setprecision(4)
          << (sample.q[j] * kRadToDeg) << ","
          << (sample.v[j] * kRadToDeg) << ","
          << (sample.acc[j] * kRadToDeg);
    }
    ofs << "\n";
  }

  ofs.close();
  std::cout << "Trajectory saved to: " << csv_path << std::endl;
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 1;
  }

  std::string tact_file = argv[1];
  std::size_t arm_dof = 14;

  if (argc >= 3) {
    arm_dof = static_cast<std::size_t>(std::stoi(argv[2]));
  }

  std::cout << "=== Tact Parser Test ===" << std::endl;
  std::cout << "Tact file: " << tact_file << std::endl;
  std::cout << "Arm DOF: " << arm_dof << std::endl;

  // Parse tact file
  leju::vr::tact_player::TactAction action;
  std::string error;
  leju::vr::tact_player::ParseOptions parse_opts;
  parse_opts.arm_dof = arm_dof;
  parse_opts.add_init_frame = true;

  if (!leju::vr::tact_player::LoadTactFile(tact_file, parse_opts, &action, &error)) {
    std::cerr << "ERROR: failed to parse tact: " << error << std::endl;
    return 1;
  }

  PrintTactAction(action, arm_dof);

  // Build interpolator
  leju::vr::tact_player::BezierInterpolator interpolator;
  leju::vr::tact_player::InterpolateOptions interp_opts;
  interp_opts.arm_dof = arm_dof;
  interp_opts.speed_scale = 1.0;

  if (!interpolator.Build(action, interp_opts, &error)) {
    std::cerr << "ERROR: failed to build interpolation: " << error << std::endl;
    return 1;
  }

  PrintTrajectory(interpolator, 0.5);

  // Save to CSV
  std::string base_name = GetBaseName(tact_file);
  std::string keyframes_csv = "/tmp/" + base_name + "_keyframes.csv";
  std::string trajectory_csv = "/tmp/" + base_name + "_trajectory.csv";

  SaveKeyframesToCsv(action, arm_dof, keyframes_csv);
  SaveTrajectoryToCsv(interpolator, trajectory_csv, 0.01);

  // Print visualization command
  std::cout << "\n=== Visualization ===" << std::endl;
  std::cout << "Run the following to plot:\n\n";
  std::cout << "python3 -c \"\n"
            << "import pandas as pd\n"
            << "import matplotlib.pyplot as plt\n"
            << "df = pd.read_csv('" << trajectory_csv << "')\n"
            << "fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)\n"
            << "for j in [7,8,9,10,11,12,13]:  # right arm\n"
            << "    axes[0].plot(df['time_s'], df[f'j{j}_pos_deg'], label=f'j{j}')\n"
            << "    axes[1].plot(df['time_s'], df[f'j{j}_vel_deg_s'], label=f'j{j}')\n"
            << "axes[0].set_ylabel('Position (deg)'); axes[0].legend(); axes[0].grid(True)\n"
            << "axes[1].set_ylabel('Velocity (deg/s)'); axes[1].set_xlabel('Time (s)'); axes[1].legend(); axes[1].grid(True)\n"
            << "plt.tight_layout(); plt.savefig('/tmp/" << base_name << "_plot.png', dpi=150); plt.show()\n"
            << "\"" << std::endl;

  return 0;
}
