cc_binary(
    name = "controller",
    srcs = [
        "angle.h",
        "controller.cc",
        "controller.h",
        "controller_board.cc",
        "controller_board.h",
        "leg.cc",
        "leg.h",
        "main.cc",
        "motor.cc",
        "motor.h",
    ],
    data = ["config/robot_config.yaml"],
    linkopts = [
        "-lpthread",
        "-lm",
    ],
    deps = ["@yaml-cpp//:yaml-cpp"],
)

sh_binary(
    name = "deploy",
    srcs = ["scripts/deploy.sh"],
    args = ["../../mcu_firmware/leg_hex.hex"],
    data = [
        "config/robot_config.yaml",
        "scripts/upload_firmware.sh",
        ":controller",
        "//src/mcu_firmware",
    ],
)

sh_binary(
    name = "deploy_calibration",
    srcs = ["scripts/deploy.sh"],
    args = ["../../mcu_firmware/calibration_hex.hex"],
    data = [
        "config/robot_config.yaml",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware:mcu_calibration",
    ],
)

sh_binary(
    name = "deploy_encoder",
    srcs = ["scripts/deploy.sh"],
    args = ["../../mcu_firmware/encoder/encoder_hex.hex"],
    data = [
        "config/robot_config.yaml",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware/encoder:mcu_encoder",
    ],
)

sh_binary(
    name = "deploy_position_test",
    srcs = ["scripts/deploy.sh"],
    args = ["../../mcu_firmware/position_test_hex.hex"],
    data = [
        "config/robot_config.yaml",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware:mcu_position_test",
    ],
)

sh_binary(
    name = "run",
    srcs = ["scripts/run.sh"],
)
