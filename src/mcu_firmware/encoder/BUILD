load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@AvrToolchain//platforms/cpu_frequency:cpu_frequency.bzl", "cpu_frequency_flag")
load("@AvrToolchain//:helpers.bzl", "generate_hex")
load("//:src/mcu_firmware/defs.bzl", "transitioning_rule")

package(default_visibility = ["//visibility:public"])

transitioning_rule(
    name = "mcu_encoder",
    actual_binary = ":encoder_hex",
)

cc_binary(
    name = "mcu_encoder_bin",
    srcs = ["main_loop.cc"],
    copts = cpu_frequency_flag() + [
        "-Os",
        "-ffunction-sections",
        "-fdata-sections",
        "-Wl,--gc-sections",
        "-Wl,--strip-all",
    ],
    deps = ["//src/mcu_firmware/base:mcu_firmware_base"],
)

generate_hex(
    name = "encoder_hex",
    input = "mcu_encoder_bin",
)
