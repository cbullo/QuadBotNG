load("@bazel_skylib//rules:common_settings.bzl", "string_flag")

string_flag(
    name = "simulator",
    build_setting_default = "0",
)

config_setting(
    name = "simulator_mode",
    flag_values = {
        ":simulator": "1",
    },
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
    # deps = "@gazebo",
)

cc_library(
    name = "controller_lib",
    srcs = [
        "angle.h",
        "bldc_driver_board.cc",
        "bldc_driver_board.h",
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
    ],
    copts = [
        "-flto",
        "-fuse-linker-plugin",
        "-ffat-lto-objects",
    ],
    defines = select({
        ":simulator_mode": {"QUADBOT_SIMULATOR": "1"},
        "//conditions:default": {},
    }),
    linkopts = [
        "-lpthread",
        "-lm",
    ],
    visibility = ["//visibility:public"],
    deps =
        select(
            {
                ":simulator_mode": [
                    ":gazebo_sim_lib",
                    "//src/libs/communication:binary_communication",
                    "@fmt",
                    "@yaml-cpp",
                ],
                "//conditions:default": [
                    "//src/libs/communication:binary_communication",
                    "@fmt",
                    "@yaml-cpp",
                ],
            },
        ),
)
