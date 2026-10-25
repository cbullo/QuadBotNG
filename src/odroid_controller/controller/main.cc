#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include "src/odroid_controller/robot_system.h"
#include "src/odroid_controller/dashboard.h"
#include "src/odroid_controller/runtime_options.h"
#include "tools/cpp/runfiles/runfiles.h"

static std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles> runfiles;
std::string ResolveAsset(const std::string& user_path, const std::string& runfile) {
  if (!user_path.empty()) return user_path;
  if (!runfiles) throw std::runtime_error("cannot locate Bazel runfiles; specify explicit asset paths");
  return runfiles->Rlocation("_main/" + runfile);
}
int main(int argc, char** argv) {
  try {
    runfiles.reset(bazel::tools::cpp::runfiles::Runfiles::Create(argv[0]));
    RuntimeOptions options;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto value = [&](const std::string& name) -> std::string {
        if (arg.rfind(name + "=", 0) == 0) return arg.substr(name.size()+1);
        if (arg == name && i+1 < argc) return argv[++i];
        throw std::invalid_argument("missing argument for " + name);
      };
      auto is = [&](const std::string& key) { return arg == key || arg.rfind(key+"=",0)==0; };
      if (is("--backend")) options.backend = value("--backend");
      else if (is("--config")) options.config = value("--config");
      else if (is("--model")) options.model = value("--model");
      else if (is("--motor-config")) options.motor_config = value("--motor-config");
      else if (is("--trace")) options.trace = value("--trace");
      else if (is("--scenario")) options.scenario = value("--scenario");
      else if (is("--duration")) options.duration = std::stod(value("--duration"));
      else if (is("--step-us")) options.step_us = std::stoi(value("--step-us"));
      else if (is("--dashboard-port")) options.dashboard_port = std::stoi(value("--dashboard-port"));
      else if (arg == "--dashboard-control") options.dashboard_control = true;
      else if (arg == "--joystick") options.joystick = true;
      else if (arg == "--no-joystick") options.no_joystick = true;
      else if (arg == "--realtime") options.realtime = true;
      else if (arg == "--suspended") options.suspended = true;
      else if (arg == "--pty-server") options.pty_server = true;
      else if (arg == "--headless") {}  // native runner is headless
      else if (arg == "--help") {
        std::cout << "--backend=serial|sim --config=PATH --duration=SECONDS --joystick\n"
                     "Simulation: --scenario=idle|hold|stand|bob|init|standup|pulse|stop --suspended --realtime\n"
                     "            --trace=CSV --step-us=500 --motor-config=PATH --pty-server\n"
                     "Dashboard: --dashboard-port=9876 [--dashboard-control] (loopback only)\n";
        return 0;
      } else throw std::invalid_argument("unknown option " + arg);
    }
    if (!std::isfinite(options.duration) || options.duration < 0 || options.step_us <= 0 ||
        5000 % options.step_us != 0) throw std::invalid_argument("duration must be nonnegative; step-us must divide 5000");
    if (options.dashboard_port < 0 || options.dashboard_port > 65535 ||
        (options.dashboard_control && !options.dashboard_port))
      throw std::invalid_argument("dashboard requires a port in 1..65535");
    if (options.dashboard_control && options.backend == "serial" && !options.no_joystick)
      throw std::invalid_argument("dashboard motion requires --no-joystick for exclusive control");
    if (options.dashboard_port && options.pty_server)
      throw std::invalid_argument("attach dashboard to the serial client, not the PTY server");
    if (options.backend == "sim") {
      if (options.dashboard_port) options.realtime = true;
      if (options.duration == 0) options.duration = options.scenario == "standup" ? 50 : options.scenario == "init" ? 30 : 10;
#ifdef QUADBOT_NATIVE_SIM
      return RunSimulation(options);
#else
      throw std::runtime_error("this hardware-only build has no simulation backend");
#endif
    }
    if (options.backend != "serial") throw std::invalid_argument("unknown backend");
    if (options.scenario != "idle" || options.pty_server || options.suspended)
      throw std::invalid_argument("scripted scenarios/PTY/suspension require --backend=sim");
    auto config = YAML::LoadFile(ResolveAsset(options.config,"src/odroid_controller/controller/config/robot_config.yaml"));
    RobotSystem robot(config, [](const auto&,const auto&) { return std::make_unique<BLDCDriverBoard>(); }, !options.no_joystick);
    Dashboard dashboard(options.dashboard_port, options.dashboard_control, "serial");
    robot.Connect();
    auto start = std::chrono::steady_clock::now(), previous = start;
    while (options.duration == 0 || std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() < options.duration) {
      robot.Pump();
      auto now = std::chrono::steady_clock::now();
      dashboard.Pump(robot, std::chrono::duration<double>(now-start).count());
      float dt = std::chrono::duration<float>(now-previous).count();
      if (dt >= 0.005f) { robot.Update(dt); previous = now; }
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    robot.Stop(); robot.Update(0.005f);
    // Flush STOP before disconnect. Hardware execution remains explicitly selected.
    for (int i = 0; i < 100; ++i) { robot.Pump(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    return 0;
  } catch (const std::exception& e) { std::cerr << "Controller: " << e.what() << '\n'; return 1; }
}
