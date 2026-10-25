load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("@rules_cc//cc:defs.bzl", "cc_library")

string_flag(
    name = "simulator",
    build_setting_default = "0",
    visibility = ["//visibility:public"],
)

config_setting(
    name = "simulator_mode",
    flag_values = {
        ":simulator": "1",
    },
    visibility = ["//visibility:public"],
)

config_setting(
    name = "hw_mode",
    flag_values = {
        ":simulator": "0",
    },
)

cc_library(
    name = "gazebo_sim_lib",
    srcs = [
        "gazebo_driver.cc",
        "gazebo_driver.h",
    ],
    tags = ["manual"],
    visibility = ["//visibility:public"],
    # deps = "@gazebo",
)

cc_library(
    name = "controller_lib",
    srcs = [
        "angle.h",
        "bldc_driver_board.cc",
        "bldc_driver_board.h",
        "boards/protocol.h",
        "boards/transport.cc",
        "boards/transport.h",
        "controller.cc",
        "controller.h",
        "event_factory.h",
        "event_ids.h",
        "joystick_input.h",
        "leg.cc",
        "leg.h",
        "leg_control.h",
        "leg_controls.cc",
        "leg_controls.h",
        "log_helper.h",
        "motor.cc",
        "motor.h",
        "pid_utils.h",
        "robot_system.cc",
        "robot_system.h",
        "runtime_options.h",
    ],
    copts = [
        "-flto",
        "-fuse-linker-plugin",
        "-ffat-lto-objects",
    ],
    linkopts = [
        "-lpthread",
        "-lm",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//src/libs/communication:binary_communication",
        "@fmt",
        "@yaml-cpp",
    ],
)

cc_library(
    name = "emulated_board",
    srcs = ["boards/emulated_board.cc"],
    hdrs = ["boards/emulated_board.h"],
    visibility = ["//visibility:public"],
    deps = [":controller_lib"],
)
