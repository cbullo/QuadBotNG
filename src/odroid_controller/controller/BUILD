load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

# Preserve the old flag spelling while sharing one setting with controller_lib.
alias(
    name = "simulator",
    actual = "//src/odroid_controller:simulator",
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
        "//src/odroid_controller:simulator_mode": ["QUADBOT_SIMULATOR=1"],
        "//conditions:default": [],
    }),
    linkopts = [
        "-lpthread",
        "-lm",
    ],
    visibility = ["//visibility:public"],
    deps =
        select(
            {
                "//src/odroid_controller:simulator_mode": [
                    "//src/libs/communication:binary_communication",
                    "//src/odroid_controller:controller_lib",
                    "//src/odroid_controller:gazebo_sim_lib",
                    "@fmt",
                    "@yaml-cpp",
                ],
                "//conditions:default": [
                    "//src/libs/communication:binary_communication",
                    "//src/odroid_controller:controller_lib",
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
    ],
)

sh_binary(
    name = "deploy_calibration",
    srcs = ["scripts/deploy.sh"],
    args = ["../../../mcu_firmware/calibration/calibration_hex.hex"],
    data = [
        "config/robot_config.yaml",
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
        "config/robot_config.yaml",
        "scripts/update_encoder.sh",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware/encoder:mcu_encoder",
    ],
)

sh_binary(
    name = "run",
    srcs = ["scripts/run.sh"],
)
