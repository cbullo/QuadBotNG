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
    linkopts = ["-lpthread"],
    deps = ["@yaml-cpp//:yaml-cpp"],
)

sh_binary(
    name = "deploy",
    srcs = ["scripts/deploy.sh"],
    data = [
        "config/robot_config.yaml",
        "scripts/upload_firmware.sh",
        ":controller",
        "//src/mcu_firmware",
    ],
)

sh_binary(
    name = "run",
    srcs = ["scripts/run.sh"],
)
