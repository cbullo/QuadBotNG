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

cc_binary(
    name = "controller",
    srcs = [
        "main.cc",
    ],
    copts = [
        "-flto",
        "-fuse-linker-plugin",
        "-ffat-lto-objects",
    ],
    data = ["config/robot_config.yaml"],
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
                    "//src/odroid_controller:controller_lib",
                    "//src/odroid_controller/web_server",
                    "@fmt",
                    "@yaml-cpp",
                ],
                "//conditions:default": [
                    "//src/libs/communication:binary_communication",
                    "//src/odroid_controller:controller_lib",
                    "//src/odroid_controller/web_server",
                    "@fmt",
                    "@yaml-cpp",
                ],
            },
        ),
)

sh_binary(
    name = "deploy_controller",
    srcs = ["scripts/deploy.sh"],
    args = ["../../../mcu_firmware/controller/controller_hex.hex"],
    data = [
        "config/robot_config.yaml",
        "scripts/update_encoder.sh",
        "scripts/upload_firmware.sh",
        ":controller",
        "//src/mcu_firmware/controller:mcu_controller",
    ] + glob(["public/**/*"]),
)

sh_binary(
    name = "deploy_calibration",
    srcs = ["scripts/deploy.sh"],
    args = ["../../../mcu_firmware/calibration/calibration_hex.hex"],
    data = [
        "controller/config/robot_config.yaml",
        "scripts/update_encoder.sh",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware/calibration:mcu_calibration",
    ],
)

sh_binary(
    name = "deploy_encoder",
    srcs = ["scripts/deploy.sh"],
    args = ["../../../mcu_firmware/encoder/encoder_hex.hex"],
    data = [
        "controller/config/robot_config.yaml",
        "scripts/update_encoder.sh",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware/encoder:mcu_encoder",
    ],
)

sh_binary(
    name = "run",
    srcs = ["scripts/run.sh"],
)
