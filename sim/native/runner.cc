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
  const bool standing_up = options.scenario == "standup";
  const bool initializing = options.scenario == "init" || standing_up;
  if (initializing && options.suspended)
    throw std::invalid_argument("init requires a floating robot resting on its belly; omit --suspended");
  const bool standing = options.scenario == "stand" || bobbing;
  const int control_period_us = standing || initializing ? 1000 : 5000;
  if (control_period_us % options.step_us != 0)
    throw std::invalid_argument("physics step must divide the scenario's control period");
  if (!standing && !initializing && options.scenario != "idle" && options.scenario != "hold" && options.scenario != "pulse" && options.scenario != "stop")
    throw std::invalid_argument("unknown simulation scenario");
  if (options.pty_server && options.scenario != "idle") throw std::invalid_argument("PTY server is controlled by external client");
  auto config = YAML::LoadFile(ResolveAsset(options.config,"src/odroid_controller/controller/config/robot_config.yaml"));
  auto settings = YAML::LoadFile(ResolveAsset(options.motor_config,"sim/motor_model.yaml"));
  simulation::MotorParameters motor;
  if (settings["model"].as<std::string>() != "sinusoidal_voltage")
    throw std::invalid_argument("motor model must be sinusoidal_voltage; migrate old torque-gain settings using sim/motor_model.yaml");
  motor.voltage_limit = settings["voltage_limit"].as<double>();
  motor.resistance_line_to_line = settings["line_to_line_resistance_ohm"].as<double>();
  motor.phase_back_emf = settings["phase_back_emf_volts_per_rad_s"].as<double>();
  motor.voltage_sign = settings["voltage_to_encoder_sign"].as<int>();
  motor.zero_counts = settings["encoder_zero_counts"].as<int>();
  motor.Validate();
  if (settings["fixed_temperature_c"])
    throw std::invalid_argument("fixed_temperature_c was replaced by thermal parameters; see sim/motor_model.yaml");
  simulation::ThermalParameters thermal;
  auto thermal_settings = settings["thermal"];
  thermal.ambient_c = thermal_settings["ambient_c"].as<double>();
  thermal.initial_c = thermal_settings["initial_c"].as<double>();
  thermal.heat_capacity_j_per_k = thermal_settings["heat_capacity_j_per_k"].as<double>();
  thermal.thermal_resistance_k_per_w = thermal_settings["thermal_resistance_k_per_w"].as<double>();
  thermal.Validate();
  auto model_path = ResolveAsset(options.model, options.suspended ? "sim/mujoco/quadbot_suspended.xml" : "sim/mujoco/quadbot.xml");
  char error[2048]{};
  std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(model_path.c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
  if (!model) throw std::runtime_error(error);
  model->opt.timestep = options.step_us * 1e-6;
  std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
  const char* pose_name = initializing ? "init" : standing ? "stand" : "stance";
  int stance = mj_name2id(model.get(),mjOBJ_KEY,pose_name);
  if (stance < 0) throw std::runtime_error(std::string("model missing keyframe: ") + pose_name);
  mj_resetDataKeyframe(model.get(),data.get(),stance);
  mj_forward(model.get(),data.get());
  if (initializing && (model->njnt == 0 || model->jnt_type[0] != mjJNT_FREE))
    throw std::runtime_error("init requires a floating chassis");
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
  if (model->nu != 12 || model->neq != 8) throw std::runtime_error("expected twelve motor actuators and eight parallelogram constraints");
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
    double limit, encoder_offset;
    simulation::MotorTemperature temperature;
  };
  std::map<std::string,double> encoder_offsets;
  // ref_z is the reduced single-turn encoder reading at the positive Z stop,
  // not an electrical zero. Match that reading in the candidate CAD model so
  // UpdateZAngle can use the same reference as hardware, without giving the
  // controller the missing rotor revolutions or the true joint coordinates.
  for (const auto& entry : config["legs"]) {
    const auto c = entry.second;
    const auto name = c["motor_Z"].as<std::string>();
    encoder_offsets[name] = (c["ref_z"].as<double>() - c["max_z"].as<double>()) /
                           robot->motors.at(name)->GetGearRatio();
  }
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
                        model->actuator_ctrlrange[2*id+1],encoder_offsets[name],simulation::MotorTemperature(thermal)});
  }
  std::array<LegConfiguration, 4> standing_pose{}, raised_pose{}, folded_pose{}, tucked_pose{};
  if (standing_up) {
    // Build reference geometry in scratch data, never using the evolving plant
    // as feedback. Homing identifies the raw encoder frame at floor contact;
    // this static CAD calibration expresses the stand keyframe in that frame.
    std::unique_ptr<mjData,decltype(&mj_deleteData)> reference(mj_makeData(model.get()),mj_deleteData);
    const int stand_key = mj_name2id(model.get(), mjOBJ_KEY, "stand");
    const int ground = mj_name2id(model.get(), mjOBJ_GEOM, "ground");
    if (stand_key < 0 || ground < 0) throw std::runtime_error("standup requires stand keyframe and ground");
    const std::array<std::string,4> cad{"RF", "LF", "LB", "RB"};
    for (int i = 0; i < 4; ++i) {
      const auto* leg = robot->controller->GetLegs().GetLeg(i);
      const std::array<const Motor*,3> motors{leg->GetMotorI(), leg->GetMotorO(), leg->GetMotorZ()};
      auto coordinates = [&]() {
        std::array<double,3> a{};
        for (int j = 0; j < 3; ++j) {
          const int id = mj_name2id(model.get(), mjOBJ_ACTUATOR, motors[j]->GetName().c_str());
          a[j] = reference->actuator_length[id] * motors[j]->GetGearRatio();
        }
        return LegConfiguration{float(-0.5 * (a[0] + a[1])),
            float(a[1] - a[0] - leg->GetZGammaDir() * (4.0/3.0) * a[2]), float(a[2])};
      };
      auto floor_contact = [&](double angle) {
        mj_resetDataKeyframe(model.get(), reference.get(), stance);
        for (const char* role : {"base_to_shoulder_", "shoulder_to_main_", "shoulder_to_crank_",
                                 "main_to_lower_", "crank_to_link_"}) {
          const int joint = mj_name2id(model.get(), mjOBJ_JOINT, (std::string(role) + cad[i]).c_str());
          if (joint < 0) throw std::runtime_error("missing standup reference joint");
          const bool driven = std::string(role).find("shoulder_to_") == 0;
          reference->qpos[model->jnt_qposadr[joint]] = driven ? angle : 0;
        }
        mj_forward(model.get(), reference.get());
        for (int c = 0; c < reference->ncon; ++c) {
          const auto& contact = reference->contact[c];
          if (contact.geom[0] != ground && contact.geom[1] != ground) continue;
          const int other = contact.geom[contact.geom[0] == ground ? 1 : 0];
          const char* name = mj_id2name(model.get(), mjOBJ_GEOM, other);
          if (name && std::string(name).find("_" + cad[i] + "_") != std::string::npos && contact.dist <= 0)
            return true;
        }
        return false;
      };
      double clear = -3 * std::acos(-1.0) / 4, touching = 0;
      if (floor_contact(clear) || !floor_contact(touching))
        throw std::runtime_error("standup floor reference is not bracketed by the CAD sweep");
      for (int n = 0; n < 30; ++n) {
        const double middle = (clear + touching) / 2;
        if (floor_contact(middle)) touching = middle; else clear = middle;
      }
      floor_contact(touching);
      const auto floor_pose = coordinates();
      const float theta_offset = leg->GetRefTheta() - floor_pose.theta;
      floor_contact(-2.5);
      raised_pose[i] = coordinates();
      raised_pose[i].theta += theta_offset;
      folded_pose[i] = raised_pose[i];
      folded_pose[i].gamma = (leg->GetGammaDir() > 0 ? leg->GetMinGamma() : leg->GetMaxGamma()) +
                            0.1f * leg->GetGammaDir();
      mj_resetDataKeyframe(model.get(), reference.get(), stand_key);
      // Take the upper, collision-free arc. The final physical pose is the
      // stand keyframe, with the two driven hinges one full turn lower.
      for (const char* role : {"shoulder_to_main_", "shoulder_to_crank_"}) {
        const int joint = mj_name2id(model.get(), mjOBJ_JOINT, (std::string(role) + cad[i]).c_str());
        reference->qpos[model->jnt_qposadr[joint]] -= 2 * std::acos(-1.0);
      }
      mj_forward(model.get(), reference.get());
      standing_pose[i] = coordinates();
      standing_pose[i].theta += theta_offset;
      tucked_pose[i] = standing_pose[i];
      tucked_pose[i].gamma = folded_pose[i].gamma;
    }
  }
  std::ofstream trace;
  std::array<int, 2> imu_addresses{};
  for (int i = 0; i < 2; ++i) {
    const char* name = i == 0 ? "imu_gyro" : "imu_accelerometer";
    const int id = mj_name2id(model.get(), mjOBJ_SENSOR, name);
    if (id < 0 || model->sensor_dim[id] != 3 ||
        model->sensor_type[id] != (i == 0 ? mjSENS_GYRO : mjSENS_ACCELEROMETER))
      throw std::runtime_error(std::string("missing or invalid IMU sensor: ") + name);
    imu_addresses[i] = model->sensor_adr[id];
  }
  const int imu_site = mj_name2id(model.get(), mjOBJ_SITE, "imu");
  if (imu_site < 0) throw std::runtime_error("missing IMU site");
  const auto sample_imu = [&](bool valid) {
    RobotSystem::ImuSample imu;
    imu.time = data->time; imu.valid = valid;
    for (int i = 0; i < 3; ++i) {
      imu.angular_velocity_rad_s[i] = data->sensordata[imu_addresses[0] + i];
      imu.specific_force_m_s2[i] = data->sensordata[imu_addresses[1] + i];
    }
    if (model->body_weldid[model->site_bodyid[imu_site]] == 0) {
      // MuJoCo reports zero acceleration on a world-welded body. A physical
      // accelerometer on the suspended chassis senses the support against g.
      mju_mulMatTVec3(imu.specific_force_m_s2.data(), data->site_xmat + 9*imu_site, model->opt.gravity);
      for (auto& value : imu.specific_force_m_s2) value = -value;
    }
    return imu;
  };
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
    trace << ",imu_valid,imu_gyro_x_rad_s,imu_gyro_y_rad_s,imu_gyro_z_rad_s"
             ",imu_accel_x_m_s2,imu_accel_y_m_s2,imu_accel_z_m_s2";
    if (initializing)
      for (const auto& [name,leg] : robot->legs) trace << ',' << name << "_init_stage";
    trace << '\n' << std::setprecision(17);
  }
  auto start = std::chrono::steady_clock::now();
  bool started = false, stopped = false;
  bool support_released = !standing;
  double support_release_time = -1;
  double homing_complete_time = -1;
  double rise_start = -1, stand_complete_time = -1, stand_settled = 0;
  double rise_phase_start = 0, waypoint_settled = 0;
  int rise_phase = 0;
  const std::array<const char*,4> rise_phases{"raise_legs", "fold_legs", "position_legs", "extend"};
  std::array<LegConfiguration,4> rise_origin{}, previous_pose{};
  boards::Time run_start = 0;
  std::array<std::array<double,3>,4> stand_targets{};
  uint64_t interleg_contacts = 0;
  std::vector<int> geom_leg(model->ngeom, -1);
  const std::array<std::string, 4> cad_legs{"_RF_", "_LF_", "_RB_", "_LB_"};
  for (int g = 0; g < model->ngeom; ++g) {
    const char* name = mj_id2name(model.get(), mjOBJ_GEOM, g);
    if (name) for (int leg = 0; leg < 4; ++leg)
      if (std::string(name).find(cad_legs[leg]) != std::string::npos) geom_leg[g] = leg;
  }
  double peak_torque = 0, max_closure = 0, max_branch_error = 0, max_ground_penetration = 0;
  const int ground_geom = mj_name2id(model.get(),mjOBJ_GEOM,"ground");
  std::vector<std::array<int,4>> linkage_coordinates;
  for (const char* cad : {"RF", "LF", "RB", "LB"}) {
    std::array<int,4> coordinates{};
    int i = 0;
    for (const char* role : {"shoulder_to_main_", "shoulder_to_crank_", "main_to_lower_", "crank_to_link_"}) {
      const int joint = mj_name2id(model.get(),mjOBJ_JOINT,(std::string(role)+cad).c_str());
      if (joint < 0) throw std::runtime_error("missing parallelogram joint");
      coordinates[i++] = model->jnt_qposadr[joint];
    }
    linkage_coordinates.push_back(coordinates);
  }
  const auto end_time = static_cast<boards::Time>(options.duration * 1000000);
  // Motor feedback crosses the board codec; the chassis IMU is a separate sensor.
  for (clock->now = 0; clock->now < end_time; clock->now += options.step_us) {
    // The startup fixture freezes coordinates without physical support forces;
    // accelerations from its unconstrained dynamics are not sensor samples.
    robot->imu = sample_imu(support_released);
    for (const auto& b : bindings)
      b.board->SetSensors(b.channel,motor.Encoder(data->actuator_length[b.actuator] + b.encoder_offset),b.temperature.Sensor());
    if (!options.pty_server) robot->Pump();
    for (auto& [name,board] : emulators) board->Pump();
    if (!options.pty_server) {
      robot->Pump();
      if (!started && robot->Ready() && !robot->boards.begin()->second->AreCommandsScheduled()) {
        started = true; run_start = clock->now;
        if (initializing && !dashboard.Overridden()) robot->StartInitialization();
        if (standing && !dashboard.Overridden()) robot->HoldRotorPose(settings["stance"]["p_volts_per_rad"].as<double>(),
            settings["stance"]["d_volts_per_rad_s"].as<double>(),motor.voltage_sign,motor.voltage_limit);
        if (bobbing)
          for (int i = 0; i < 4; ++i) stand_targets[i] = robot->rotor_hold_controls[i].GetTarget();
        if (!dashboard.Overridden() && (options.scenario == "hold" || options.scenario == "stop")) robot->HoldCurrentPose();
        if (!dashboard.Overridden() && options.scenario == "pulse")
          for (auto& [name,board] : robot->boards) board->SendSetCommand(0,CMD_STATE,uint8_t(3));
      }
      if (clock->now % control_period_us == 0) {
        if (standing_up && robot->InitializationComplete() &&
            !dashboard.Overridden() && robot->GetPoseFault().empty()) {
          if (rise_start < 0) {
            rise_phase_start = rise_start = clock->now * 1e-6;
            for (int i = 0; i < 4; ++i)
              previous_pose[i] = rise_origin[i] = robot->controller->GetLegs().GetLeg(i)->GetState();
          }
          const double elapsed = clock->now * 1e-6 - rise_phase_start;
          const std::array<double,4> durations{1.5, 2.0, 4.0, 4.0};
          const std::array<const std::array<LegConfiguration,4>*,5> waypoints{
              &rise_origin, &raised_pose, &folded_pose, &tucked_pose, &standing_pose};
          const int phase = rise_phase;
          const double t = std::clamp(elapsed / durations[phase], 0.0, 1.0);
          const double blend = t*t*t*(10 + t*(-15 + 6*t));
          std::array<LegConfiguration,4> targets{};
          bool settled = phase == 3 && t == 1;
          bool waypoint_reached = t == 1;
          for (int i = 0; i < 4; ++i) {
            auto lerp = [blend](float a, float b) { return float(a + blend * (b-a)); };
            const auto& from = (*waypoints[phase])[i];
            const auto& to = (*waypoints[phase+1])[i];
            targets[i] = {lerp(from.theta, to.theta), lerp(from.gamma, to.gamma), lerp(from.z, to.z)};
            const auto q = robot->controller->GetLegs().GetLeg(i)->GetState();
            for (auto pair : {std::pair<float,float>{q.theta, targets[i].theta},
                              {q.gamma, targets[i].gamma}, {q.z, targets[i].z}})
            {
              settled &= std::abs(pair.first - pair.second) < 0.1f;
              waypoint_reached &= std::abs(pair.first - pair.second) < 0.04f;
            }
            for (auto pair : {std::pair<float,float>{q.theta, previous_pose[i].theta},
                              {q.gamma, previous_pose[i].gamma}, {q.z, previous_pose[i].z}})
              settled &= std::abs(pair.first - pair.second) < 0.0005f;
            previous_pose[i] = q;
          }
          robot->SetStandingTargets(targets);
          stand_settled = settled ? stand_settled + control_period_us * 1e-6 : 0;
          if (stand_settled >= 0.5 && stand_complete_time < 0) stand_complete_time = clock->now * 1e-6;
          waypoint_settled = waypoint_reached ? waypoint_settled + control_period_us * 1e-6 : 0;
          if (phase < 3 && waypoint_settled >= 0.12) {
            ++rise_phase;
            rise_phase_start = clock->now * 1e-6;
            waypoint_settled = 0;
          } else if (elapsed > durations[phase] + 5.0 && stand_complete_time < 0) {
            robot->AbortPoseMotion(std::string("standup: ") + rise_phases[phase] +
                                  " did not reach target within effort limits");
          }
        }
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
      b.temperature.Step(motor.CopperLoss(data->ctrl[b.actuator]),model->opt.timestep);
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
    for (const auto& q : linkage_coordinates) {
      const double delta = data->qpos[q[1]] - data->qpos[q[0]];
      for (int i : {2, 3}) {
        const double error = std::abs(data->qpos[q[i]] - delta);
        max_branch_error = std::max(max_branch_error,error);
        if (error > 0.05) throw std::runtime_error("parallelogram left intended branch (angle error > 0.05 rad)");
      }
    }
    for (int i = 0; i < data->ncon; ++i) {
      const auto& contact = data->contact[i];
      if (contact.geom[0] == ground_geom || contact.geom[1] == ground_geom)
        max_ground_penetration = std::max(max_ground_penetration, -contact.dist);
      const int a = geom_leg[contact.geom[0]], b = geom_leg[contact.geom[1]];
      if (a >= 0 && b >= 0 && a != b && contact.dist < 0) ++interleg_contacts;
    }
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
    if (initializing && homing_complete_time < 0 && robot->InitializationComplete())
      homing_complete_time = data->time;
    if (trace && clock->now % 5000 == 0) {
      trace << data->time;
      for (int i = 0; i < model->nq; ++i) trace << ',' << data->qpos[i];
      for (const auto& b : bindings) trace << ',' << data->ctrl[b.actuator];
      for (const auto& [name,instance] : robot->motors) trace << ',' << instance->GetRawAngle();
      for (const auto& [name,instance] : robot->motors) trace << ',' << instance->GetTemperature();
      for (const auto& b : bindings) trace << ',' << b.temperature.Celsius();
      trace << ',' << !support_released;
      trace << ',' << !robot->GetPoseFault().empty();
      const auto imu = sample_imu(support_released);
      trace << ',' << imu.valid;
      for (double value : imu.angular_velocity_rad_s) trace << ',' << value;
      for (double value : imu.specific_force_m_s2) trace << ',' << value;
      if (initializing) {
        for (const auto& [name,leg] : robot->legs) {
          const auto* control = dynamic_cast<const InitializationLegControl*>(leg->GetControl());
          trace << ',' << (control ? static_cast<int>(control->GetStage()) : -1);
        }
      }
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
            << " max_branch_error_rad=" << max_branch_error << " max_ground_penetration_m=" << max_ground_penetration
            << " watchdog_resets=" << watchdog_resets << " thermal_resets=" << thermal_resets
            << " pose_fault=" << !robot->GetPoseFault().empty() << '\n';
  if (initializing)
    std::cout << "homing_complete=" << (homing_complete_time >= 0)
              << " interleg_contacts=" << interleg_contacts
              << " homing_complete_at=" << homing_complete_time << '\n';
  if (standing_up)
    std::cout << "standup_started_at=" << rise_start << " standup_phase=" << rise_phases[rise_phase] << " stand_complete_at=" << stand_complete_time << '\n';
  if (standing || standing_up) {
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
