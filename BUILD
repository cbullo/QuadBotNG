# load("@com_grail_bazel_compdb//:aspects.bzl", "compilation_database")

# compilation_database(
#     name = "example_compdb",
#     targets = [
#         "//leg_firmware:leg",
#     ],
#     # ideally should be the same as `bazel info execution_root`.
#     exec_root = "/home/tomasz/.cache/bazel/_bazel_tomasz/4c19095c8d368009c5130f5846cef6ca/execroot/__main__",
# )

platform(
    name = "RCTimer",
    constraint_values = [
        "@AvrToolchain//platforms/mcu:atmega328p",
        "@AvrToolchain//platforms/cpu_frequency:16mhz",
        "@AvrToolchain//platforms/misc:hardware_uart",
    ],
    parents = ["@AvrToolchain//platforms:avr_common"],
)

platform(
    name = "Odroid-C4",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux"
    ]
)