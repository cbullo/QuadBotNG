load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@AvrToolchain//platforms/cpu_frequency:cpu_frequency.bzl", "cpu_frequency_flag")
load("@AvrToolchain//:helpers.bzl", "generate_hex")
load("//:src/mcu_firmware/defs.bzl", "transitioning_rule")

package(default_visibility = ["//visibility:public"])

transitioning_rule(
    name = "mcu_controller",
    actual_binary = ":controller_hex",
)

cc_binary(
    name = "mcu_controller_bin",
    srcs = ["main_loop.cc"],
    copts = cpu_frequency_flag() + [
        "-Os",
        "-ffunction-sections",
        "-fdata-sections",
        "-ffat-lto-objects",
        "-fuse-linker-plugin",
        "-flto",
        "-Wl,--gc-sections",
        "-Wl,--strip-all",
    ],
    deps = [
        "//src/libs/communication:binary_communication",
        "//src/mcu_firmware/base:mcu_firmware_base",
    ],
)

generate_hex(
    name = "controller_hex",
    input = "mcu_controller_bin",
)
