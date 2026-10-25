#include <mujoco/mujoco.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <thread>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include "motor_plant.h"
#include "src/odroid_controller/boards/emulated_board.h"
#include "src/odroid_controller/robot_system.h"
#include "src/odroid_controller/dashboard.h"
#include "src/odroid_controller/runtime_options.h"

int RunSimulation(const RuntimeOptions& options) {
  if (options.joystick) throw std::invalid_argument("simulation uses scripted events; joystick not enabled");
  if (options.scenario == "homing")
    throw std::invalid_argument("homing unavailable: mechanical stops and absolute encoder references are not validated");
  const bool bobbing = options.scenario == "bob";
  const bool standing = options.scenario == "stand" || bobbing;
  const int control_period_us = standing ? 1000 : 5000;
  if (control_period_us % options.step_us != 0)
    throw std::invalid_argument("physics step must divide the scenario's control period");
  if (!standing && options.scenario != "idle" && options.scenario != "hold" && options.scenario != "pulse" && options.scenario != "stop")
    throw std::invalid_argument("unknown simulation scenario");
  if (options.pty_server && options.scenario != "idle") throw std::invalid_argument("PTY server is controlled by external client");
  auto config = YAML::LoadFile(ResolveAsset(options.config,"src/odroid_controller/controller/config/robot_config.yaml"));
  auto settings = YAML::LoadFile(ResolveAsset(options.motor_config,"sim/motor_model.yaml"));
  simulation::MotorParameters motor;
  if (settings["model"].as<std::string>() != "linear_voltage_back_emf") throw std::invalid_argument("unknown motor model");
  motor.voltage_limit = settings["voltage_limit"].as<double>();
  motor.torque_per_volt = settings["shaft_torque_per_volt"].as<double>();
  motor.back_emf = settings["back_emf_volts_per_rad_s"].as<double>();
  motor.voltage_sign = settings["voltage_to_encoder_sign"].as<int>();
  motor.zero_counts = settings["encoder_zero_counts"].as<int>();
  motor.Validate();
  if (settings["fixed_temperature_c"])
    throw std::invalid_argument("fixed_temperature_c was replaced by thermal parameters; see sim/motor_model.yaml");
  simulation::ThermalParameters thermal;
  auto thermal_settings = settings["thermal"];
  thermal.ambient_c = thermal_settings["ambient_c"].as<double>();
  thermal.initial_c = thermal_settings["initial_c"].as<double>();
  thermal.resistance_ohm = thermal_settings["effective_resistance_ohm"].as<double>();
  thermal.heat_capacity_j_per_k = thermal_settings["heat_capacity_j_per_k"].as<double>();
  thermal.thermal_resistance_k_per_w = thermal_settings["thermal_resistance_k_per_w"].as<double>();
  thermal.Validate();
  auto model_path = ResolveAsset(options.model, options.suspended ? "sim/mujoco/quadbot_suspended.xml" : "sim/mujoco/quadbot.xml");
  char error[2048]{};
  std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(model_path.c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
  if (!model) throw std::runtime_error(error);
  model->opt.timestep = options.step_us * 1e-6;
  std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
  const char* pose_name = standing ? "stand" : "stance";
  int stance = mj_name2id(model.get(),mjOBJ_KEY,pose_name);
  if (stance < 0) throw std::runtime_error(std::string("model missing keyframe: ") + pose_name);
  mj_resetDataKeyframe(model.get(),data.get(),stance);
  mj_forward(model.get(),data.get());
  if (standing && !options.suspended) {
    // Place the known stance on the floor before boot, rather than dropping
    // the unpowered robot from the demo's airborne keyframe.
    if (model->njnt == 0 || model->jnt_type[0] != mjJNT_FREE)
      throw std::runtime_error("standing scenario requires a floating chassis");
    for (int i = 0; data->ncon == 0 && i < 1000; ++i) {
      data->qpos[2] -= 0.0005;
      mj_forward(model.get(),data.get());
    }
    if (data->ncon == 0) throw std::runtime_error("standing pose could not reach ground");
    data->qpos[2] += 0.0005;
    mj_forward(model.get(),data.get());
  }
  if (model->nu != 12 || model->neq != 4) throw std::runtime_error("expected twelve motor actuators and four closures");
  std::map<std::string,double> bob_offsets;
  if (bobbing) {
    // A small symmetric knee bend: main +0.09, crank -0.09 rad.
    // Convert this prescribed CAD pose to rotor excursions with the model's
    // existing three-motor differential, including each leg's signs/gearing.
    // This scratch data defines targets only; feedback still comes from boards.
    std::unique_ptr<mjData,decltype(&mj_deleteData)> target(mj_makeData(model.get()),mj_deleteData);
    mj_resetDataKeyframe(model.get(),target.get(),stance);
    for (const char* cad : {"RF","LF","RB","LB"}) {
      for (const auto& [prefix, offset] : std::map<std::string,double>{
               {"shoulder_to_main_", 0.09}, {"shoulder_to_crank_", -0.09}}) {
        const int joint = mj_name2id(model.get(),mjOBJ_JOINT,(prefix+cad).c_str());
        if (joint < 0) throw std::runtime_error("missing bob trajectory joint");
        target->qpos[model->jnt_qposadr[joint]] += offset;
      }
    }
    mj_forward(model.get(),target.get());
    for (int i = 0; i < model->nu; ++i)
      bob_offsets[mj_id2name(model.get(),mjOBJ_ACTUATOR,i)] = target->actuator_length[i]-data->actuator_length[i];
  }
  auto clock = std::make_shared<boards::ManualClock>();
  std::map<std::string,std::unique_ptr<boards::EmulatedBoard>> emulators;
  std::map<std::string,std::shared_ptr<boards::MemoryLink>> links;
  auto factory = [&](const std::string& name,const YAML::Node&) {
    auto link = std::make_shared<boards::MemoryLink>();
    link->baud = settings["serial_baud"].as<unsigned>();
    link->latency_us = settings["transport_latency_us"].as<int>();
    if (link->latency_us < 0) throw std::invalid_argument("negative transport latency");
    links[name] = link;
    emulators[name] = std::make_unique<boards::EmulatedBoard>(
        std::make_unique<boards::MemoryTransport>(link,1,clock),clock,link);
    return std::make_unique<BLDCDriverBoard>(std::make_unique<boards::MemoryTransport>(link,0,clock),clock);
  };
  auto robot = std::make_unique<RobotSystem>(config,factory);
  Dashboard dashboard(options.dashboard_port, options.dashboard_control, "sim");
  if (robot->boards.size() != 6 || robot->motors.size() != 12) throw std::runtime_error("expected six two-channel boards");
  if (options.pty_server) {
    emulators.clear();
    YAML::Node overlay = YAML::Clone(config);
    for (auto& [name,board] : robot->boards) {
      int fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0 || grantpt(fd) || unlockpt(fd)) throw std::runtime_error("cannot create PTY");
      overlay["controllers"][name]["controller_address"] = std::string(ptsname(fd));
      emulators[name] = std::make_unique<boards::EmulatedBoard>(std::make_unique<boards::SerialTransport>(fd,clock),clock);
    }
    if (options.trace.empty()) throw std::invalid_argument("PTY server requires --trace=temporary-config.yaml for device overlay");
    const auto temporary_overlay = options.trace + ".tmp";
    std::ofstream overlay_file(temporary_overlay);
    if (!overlay_file) throw std::runtime_error("cannot write PTY configuration overlay");
    overlay_file << overlay;
    overlay_file.close();
    if (!overlay_file || std::rename(temporary_overlay.c_str(),options.trace.c_str()) != 0)
      throw std::runtime_error("cannot publish PTY configuration overlay");
    std::cout << "PTY configuration: " << options.trace << std::endl;
  } else robot->Connect();
  struct Binding {
    int actuator, channel;
    boards::EmulatedBoard* board;
    double limit;
    simulation::MotorTemperature temperature;
  };
  std::vector<Binding> bindings;
  std::set<int> seen;
  for (const auto& [name,instance] : robot->motors) {
    int id = mj_name2id(model.get(),mjOBJ_ACTUATOR,name.c_str());
    if (id < 0 || !seen.insert(id).second) throw std::runtime_error("missing/duplicate MJCF motor " + name);
    if (model->actuator_trntype[id] != mjTRN_TENDON || model->actuator_biastype[id] != mjBIAS_NONE ||
        model->actuator_gainprm[id*mjNGAIN] != 1 || !model->actuator_ctrllimited[id])
      throw std::runtime_error("expected plain torque-driven motor actuator");
    auto c = config["motors"][name];
    bindings.push_back({id,instance->GetIndex(),emulators.at(c["controller"].as<std::string>()).get(),
                        model->actuator_ctrlrange[2*id+1],simulation::MotorTemperature(thermal)});
  }
  std::ofstream trace;
  if (!options.trace.empty() && !options.pty_server) {
    trace.open(options.trace);
    if (!trace) throw std::runtime_error("cannot write trace");
    trace << "time";
    for (int i = 0; i < model->nq; ++i) trace << ",q" << i;
    for (const auto& b : bindings) trace << "," << mj_id2name(model.get(),mjOBJ_ACTUATOR,b.actuator) << "_torque";
    for (const auto& [name,instance] : robot->motors) trace << "," << name << "_encoder";
    for (const auto& [name,instance] : robot->motors) trace << "," << name << "_temperature_c";
    for (const auto& b : bindings) trace << "," << mj_id2name(model.get(),mjOBJ_ACTUATOR,b.actuator) << "_plant_temperature_c";
    trace << ",startup_support,pose_fault";
    trace << '\n' << std::setprecision(17);
  }
  auto start = std::chrono::steady_clock::now();
  bool started = false, stopped = false;
  bool support_released = !standing;
  double support_release_time = -1;
  boards::Time run_start = 0;
  std::array<std::array<double,3>,4> stand_targets{};
  double peak_torque = 0, max_closure = 0;
  const auto end_time = static_cast<boards::Time>(options.duration * 1000000);
  // No direct pose feedback to the C++ controller: all samples cross the byte codec.
  for (clock->now = 0; clock->now < end_time; clock->now += options.step_us) {
    for (const auto& b : bindings)
      b.board->SetSensors(b.channel,motor.Encoder(data->actuator_length[b.actuator]),b.temperature.Sensor());
    if (!options.pty_server) robot->Pump();
    for (auto& [name,board] : emulators) board->Pump();
    if (!options.pty_server) {
      robot->Pump();
      if (!started && robot->Ready() && !robot->boards.begin()->second->AreCommandsScheduled()) {
        started = true; run_start = clock->now;
        if (standing && !dashboard.Overridden()) robot->HoldRotorPose(settings["stance"]["p_volts_per_rad"].as<double>(),
            settings["stance"]["d_volts_per_rad_s"].as<double>(),motor.voltage_sign,motor.voltage_limit);
        if (bobbing)
          for (int i = 0; i < 4; ++i) stand_targets[i] = robot->rotor_hold_controls[i].GetTarget();
        if (!dashboard.Overridden() && (options.scenario == "hold" || options.scenario == "stop")) robot->HoldCurrentPose();
        if (!dashboard.Overridden() && options.scenario == "pulse")
          for (auto& [name,board] : robot->boards) board->SendSetCommand(0,CMD_STATE,uint8_t(3));
      }
      if (clock->now % control_period_us == 0) {
        if (bobbing && started && support_released && !dashboard.Overridden() && robot->GetPoseFault().empty()) {
          // Settle for one second, then smoothly crouch and rise every four
          // seconds. Never extend beyond the original stand target.
          const double elapsed = std::max(0.0, clock->now*1e-6-support_release_time-1.0);
          const double blend = 0.5*(1.0-std::cos(2.0*std::acos(-1.0)*elapsed/4.0));
          for (int i = 0; i < 4; ++i) {
            auto* leg = robot->controller->GetLegs().GetLeg(i);
            std::array<Motor*,3> motors{leg->GetMotorI(),leg->GetMotorO(),leg->GetMotorZ()};
            auto target = stand_targets[i];
            for (int j = 0; j < 3; ++j) target[j] += blend*bob_offsets.at(motors[j]->GetName());
            robot->rotor_hold_controls[i].SetTarget(target);
          }
        }
        if (started && !dashboard.Overridden() && options.scenario == "pulse") {
          for (auto& [name,instance] : robot->motors) {
            // Exercise all mapped channels with small, finite voltage pulses.
            const double voltage = clock->now-run_start < 200000 ? 0.1 : 0;
            instance->GetController()->SendSetCommand(instance->GetIndex(),CMD_MOTOR_VOLTAGE,int16_t(voltage*512));
          }
        }
        if (started && !stopped && !dashboard.Overridden() && options.scenario == "stop" && clock->now-run_start >= 1000000) {
          robot->Stop(); stopped = true;
        }
        dashboard.Pump(*robot, clock->now * 1e-6);
        robot->Update(control_period_us * 1e-6f);
      }
      if (!started && clock->now > 6000000) throw std::runtime_error("board startup did not finish");
    }
    for (auto& b : bindings) {
      const auto& channel = b.board->channel(b.channel);
      bool enabled = b.board->state() == boards::State::Running && (channel.availability & 1) && channel.motion_type == 0;
      data->ctrl[b.actuator] = motor.Torque(b.board->Voltage(b.channel),data->actuator_velocity[b.actuator],b.limit,enabled);
      peak_torque = std::max(peak_torque,std::abs(data->ctrl[b.actuator]));
      b.temperature.Step(data->ctrl[b.actuator],motor.torque_per_volt,model->opt.timestep);
    }
    if (standing && started && !support_released) {
      support_released = std::all_of(emulators.begin(),emulators.end(),[](const auto& entry) {
        return entry.second->state() == boards::State::Running;
      });
      if (support_released) support_release_time = data->time;
    }
    // A dashboard STOP before startup must also release the explicit fixture;
    // it must not cause a later scripted RUNNING command or freeze the robot.
    if (dashboard.Overridden() && !support_released) {
      support_released = true; support_release_time = data->time;
    }
    // An explicit startup fixture holds the keyframe while the real protocol
    // boots. Once released, physics is never reset or supported by the runner.
    if (support_released) mj_step(model.get(),data.get());
    else data->time += model->opt.timestep;
    mj_forward(model.get(),data.get());
    for (const char* name : {"FR","FL","BR","BL"}) {
      auto a = mj_name2id(model.get(),mjOBJ_SITE,(std::string("closure_link_")+name).c_str());
      auto b = mj_name2id(model.get(),mjOBJ_SITE,(std::string("closure_lower_")+name).c_str());
      if (a < 0 || b < 0) throw std::runtime_error("missing closure sites");
      double error2 = 0;
      for (int k = 0; k < 3; ++k) error2 += std::pow(data->site_xpos[3*a+k]-data->site_xpos[3*b+k],2);
      max_closure = std::max(max_closure,std::sqrt(error2));
    }
    for (int i = 0; i < mjNWARNING; ++i)
      if (data->warning[i].number) throw std::runtime_error("MuJoCo warning " + std::to_string(i));
    for (int i = 0; i < model->nq; ++i) if (!std::isfinite(data->qpos[i])) throw std::runtime_error("non-finite physics state");
    if (trace && clock->now % 5000 == 0) {
      trace << data->time;
      for (int i = 0; i < model->nq; ++i) trace << ',' << data->qpos[i];
      for (const auto& b : bindings) trace << ',' << data->ctrl[b.actuator];
      for (const auto& [name,instance] : robot->motors) trace << ',' << instance->GetRawAngle();
      for (const auto& [name,instance] : robot->motors) trace << ',' << instance->GetTemperature();
      for (const auto& b : bindings) trace << ',' << b.temperature.Celsius();
      trace << ',' << !support_released;
      trace << ',' << !robot->GetPoseFault().empty();
      trace << '\n';
      if (options.realtime) trace.flush();
    }
    if (options.realtime || options.pty_server)
      std::this_thread::sleep_until(start+std::chrono::microseconds(clock->now+options.step_us));
  }
  uint64_t commands = 0, watchdog_resets = 0, thermal_resets = 0;
  for (auto& [name,board] : emulators) {
    commands += board->commands();
    watchdog_resets += board->watchdog_resets();
    thermal_resets += board->thermal_resets();
  }
  std::cout << std::setprecision(9) << "scenario=" << options.scenario << " time=" << data->time
            << " initialized=" << started << " commands=" << commands << " peak_shaft_torque=" << peak_torque
            << " max_closure_m=" << max_closure << " contacts=" << data->ncon
            << " watchdog_resets=" << watchdog_resets << " thermal_resets=" << thermal_resets
            << " pose_fault=" << !robot->GetPoseFault().empty() << '\n';
  if (standing) {
    const int ground = mj_name2id(model.get(),mjOBJ_GEOM,"ground");
    std::set<int> touching;
    for (int i = 0; i < data->ncon; ++i) {
      const auto& contact = data->contact[i];
      if (contact.geom[0] == ground) touching.insert(model->geom_bodyid[contact.geom[1]]);
      if (contact.geom[1] == ground) touching.insert(model->geom_bodyid[contact.geom[0]]);
    }
    int feet = 0;
    for (const char* name : {"RF","LF","RB","LB"})
      feet += touching.count(mj_name2id(model.get(),mjOBJ_BODY,(std::string("arm_lower_")+name+"_1").c_str()));
    std::cout << "startup_support_released_at=" << support_release_time << " feet_on_ground=" << feet << '\n';
  }
  if (!options.pty_server && !started) throw std::runtime_error("run too short to complete startup");
  return 0;
}
