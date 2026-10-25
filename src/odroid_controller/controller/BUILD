load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

# The simulator checks its transmission/board mapping against this source.
exports_files(
    ["config/robot_config.yaml"],
    visibility = [
        "//sim:__pkg__",
        "//src/odroid_controller/tests:__pkg__",
    ],
)

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
        ":pc_backend": ["QUADBOT_NATIVE_SIM=1"],
        "//conditions:default": [],
    }),
    linkopts = [
        "-lpthread",
        "-lm",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//src/odroid_controller:controller_lib",
        "@bazel_tools//tools/cpp/runfiles",
    ] + select({
        ":pc_backend": ["//sim:native_runner"],
        "//conditions:default": [],
    }),
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

config_setting(
    name = "pc_backend",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
)
