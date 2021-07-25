load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@AvrToolchain//platforms/cpu_frequency:cpu_frequency.bzl", "cpu_frequency_flag")
load("@AvrToolchain//:helpers.bzl", "generate_hex")
load("defs.bzl", "transitioning_rule")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mcu_firmware_lib",
    srcs = [
        "custom_magnetic_sensor_i2c.cc",
        "custom_magnetic_sensor_i2c.h",
    ],
    hdrs = glob(["*.h"]),
    copts = cpu_frequency_flag(),
    deps = [
        "@arduino//:arduino_eeprom_lib",
        "@arduino//:arduino_wire_lib",
        "@simplefoc//:simplefoc_lib",
    ],
)

transitioning_rule(
    name = "mcu_firmware",
    actual_binary = ":mcu_firmware_bin",
)

cc_binary(
    name = "mcu_firmware_bin",
    srcs = ["main.cc"],
    copts = cpu_frequency_flag(),
    deps = [":mcu_firmware_lib"],
)

generate_hex(
    name = "leg_hex",
    input = "mcu_firmware",
)
