load("@rules_cc//cc:defs.bzl", "cc_library")
load("@AvrToolchain//platforms/cpu_frequency:cpu_frequency.bzl", "cpu_frequency_flag")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mcu_firmware_base",
    srcs = [
        "analog_reader.cc",
        "communication.cc",
        "custom_magnetic_sensor_i2c.cc",
        "main.cc",
    ],
    hdrs = [
        "analog_reader.h",
        "communication.h",
        "custom_magnetic_sensor_i2c.h",
        "debug_led.h",
        "piecewise_linear.h",
        "temperature.h",
    ],
    copts = cpu_frequency_flag() + [
        "-Os",
        "-ffunction-sections",
        "-fdata-sections",
        "-Wl,--gc-sections",
        "-Wl,--strip-all",
    ],
    strip_include_prefix = "//src/mcu_firmware",
    deps = [
        "@arduino//:arduino_core",
        "@arduino//:arduino_eeprom_lib",
        "@arduino//:arduino_wire_lib",
        "@simplefoc//:simplefoc_lib",
    ],
)
