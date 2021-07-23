load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@AvrToolchain//platforms/cpu_frequency:cpu_frequency.bzl", "cpu_frequency_flag")
load("@AvrToolchain//:helpers.bzl", "generate_hex", "upload")

cc_library(
    name = "mcu_firmware_lib",
    srcs = [
        "custom_magnetic_sensor_i2c.h",
        "custom_magnetic_sensor_i2c.cc",
    ],
    hdrs = glob(["*.h"]),
    copts = cpu_frequency_flag(),
    deps = [
        "@arduino//:arduino_eeprom_lib",
        "@arduino//:arduino_wire_lib",
        "@simplefoc//:simplefoc_lib"
    ],
)

cc_binary(
    name = "mcu_firmware",
    srcs = ["main.cc"],
    copts = cpu_frequency_flag(),
    deps = [":mcu_firmware_lib"],
    visibility = ["//visibility:public"],
)

generate_hex(
    name = "leg_hex",
    input = "mcu_firmware",
)