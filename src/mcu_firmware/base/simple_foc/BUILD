load("@rules_cc//cc:defs.bzl", "cc_library")
load("@AvrToolchain//platforms/cpu_frequency:cpu_frequency.bzl", "cpu_frequency_flag")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simplefoc_custom",
    srcs = [
        "BLDCDriver3PWM.cc",
        "atmega328_mcu.cc",
        "bldc_motor.cc",
        "custom_magnetic_sensor_i2c.cc",
        "time_utils.cc",
    ],
    hdrs = [
        "BLDCDriver3PWM.h",
        "bldc_motor.h",
        "custom_magnetic_sensor_i2c.h",
        "default_configuration.h",
        "hardware_api.h",
        "piecewise_linear.h",
        "sin_approx.h",
        "time_utils.h",
        "types.h",
    ],
    copts = cpu_frequency_flag() + [
        "-Os",
        "-ffunction-sections",
        "-fdata-sections",
        "-flto",
        "-ffat-lto-objects",
        "-fuse-linker-plugin",
        "-nodevicelib",
        "-Wl,--gc-sections",
        "-Wl,--strip-all",
    ],
    strip_include_prefix = "//src/mcu_firmware",
    deps = [
        "@arduino//:arduino_core",
        "@arduino//:arduino_eeprom_lib",
        "@arduino//:arduino_wire_lib",
    ],
)
