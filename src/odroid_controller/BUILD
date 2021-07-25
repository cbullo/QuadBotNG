cc_binary(
    name = "controller",
    srcs = ["main.cc"],
)

sh_binary(
    name = "deploy",
    srcs = ["scripts/deploy.sh"],
    data = [
        ":controller",
        "scripts/upload_firmware.sh",
        "//src/mcu_firmware:mcu_firmware"
    ],
)
