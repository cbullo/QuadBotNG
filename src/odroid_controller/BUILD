cc_binary(
    name = "controller",
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
        "main.cc",
        "motor.cc",
        "motor.h",
    ],
    copts = [
        "-flto",
        "-fuse-linker-plugin",
        "-ffat-lto-objects",
    ],
    data = ["config/robot_config.yaml"],
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

sh_binary(
    name = "deploy_controller",
    srcs = ["scripts/deploy.sh"],
    args = ["../../mcu_firmware/controller/controller_hex.hex"],
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
    args = ["../../mcu_firmware/calibration/calibration_hex.hex"],
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
    args = ["../../mcu_firmware/encoder/encoder_hex.hex"],
    data = [
        "config/robot_config.yaml",
        "scripts/update_encoder.sh",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware/encoder:mcu_encoder",
    ],
)

sh_binary(
    name = "deploy_open_loop",
    srcs = ["scripts/deploy.sh"],
    args = ["../../mcu_firmware/open_loop/open_loop_hex.hex"],
    data = [
        "config/robot_config.yaml",
        "scripts/update_encoder.sh",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware/open_loop:mcu_open_loop",
    ],
)

sh_binary(
    name = "deploy_init_foc",
    srcs = ["scripts/deploy.sh"],
    args = ["../../mcu_firmware/init_foc/init_foc_hex.hex"],
    data = [
        "config/robot_config.yaml",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware/init_foc:mcu_init_foc",
    ],
)

sh_binary(
    name = "deploy_calibration_foc",
    srcs = ["scripts/deploy.sh"],
    args = ["../../mcu_firmware/calibration/calibration_hex.hex"],
    data = [
        "config/robot_config.yaml",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware/init_foc:mcu_calibration",
    ],
)

sh_binary(
    name = "run",
    srcs = ["scripts/run.sh"],
)
