cc_binary(
    name = "controller",
    srcs = ["main.cc"],
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
