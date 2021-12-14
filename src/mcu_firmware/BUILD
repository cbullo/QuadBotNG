load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@AvrToolchain//platforms/cpu_frequency:cpu_frequency.bzl", "cpu_frequency_flag")
load("@AvrToolchain//:helpers.bzl", "generate_hex")
load("defs.bzl", "transitioning_rule")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mcu_firmware_lib",
    srcs = [
        "analog_reader.h",
        "analog_reader.cc",
        "custom_magnetic_sensor_i2c.cc",
        "custom_magnetic_sensor_i2c.h",
    ],
    hdrs = glob(["*.h"]),
    copts = cpu_frequency_flag() + ["-Os", "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections", "-Wl,--strip-all"],
    deps = [
        "@arduino//:arduino_core",
        "@arduino//:arduino_eeprom_lib",
        "@arduino//:arduino_wire_lib",
        "@simplefoc//:simplefoc_lib",
    ],
)

###############################################################################

transitioning_rule(
    name = "mcu_firmware",
    actual_binary = ":leg_hex",
)

cc_binary(
    name = "mcu_firmware_bin",
    srcs = ["main.cc"],
    copts = cpu_frequency_flag() + ["-Os", "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections", "-Wl,--strip-all"],
    #linkopts = ["--gc-sections"],
    deps = [":mcu_firmware_lib"],
)

generate_hex(
    name = "leg_hex",
    input = "mcu_firmware_bin",
)

###############################################################################

transitioning_rule(
    name = "mcu_calibration",
    actual_binary = ":calibration_hex",
)

cc_binary(
    name = "mcu_calibration_bin",
    srcs = ["utils/calibration/main.cc"],
    copts = cpu_frequency_flag() + ["-Os", "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections", "-Wl,--strip-all"],
    #linkopts = ["--gc-sections"],
    deps = [":mcu_firmware_lib"],
)

generate_hex(
    name = "calibration_hex",
    input = "mcu_calibration_bin",
)

###############################################################################

transitioning_rule(
    name = "mcu_encoder",
    actual_binary = ":encoder_hex",
)

cc_binary(
    name = "mcu_encoder_bin",
    srcs = ["utils/encoder/main.cc"],
    copts = cpu_frequency_flag() + ["-Os", "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections", "-Wl,--strip-all"],
    #linkopts = ["--gc-sections"],
    deps = [":mcu_firmware_lib"],
)

generate_hex(
    name = "encoder_hex",
    input = "mcu_encoder_bin",
)

###############################################################################

transitioning_rule(
    name = "mcu_position_test",
    actual_binary = ":position_test_hex",
)

cc_binary(
    name = "mcu_position_test_bin",
    srcs = ["utils/position_test/main.cc"],
    copts = cpu_frequency_flag() + ["-Os", "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections", "-Wl,--strip-all"],
    #linkopts = ["--gc-sections"],
    deps = [":mcu_firmware_lib"],
)

generate_hex(
    name = "position_test_hex",
    input = "mcu_position_test_bin",
)
