## Replace workspace_name and dir_path as per your setup.
load("@com_grail_bazel_compdb//:defs.bzl", "compilation_database")
load("@com_grail_bazel_output_base_util//:defs.bzl", "OUTPUT_BASE")

compilation_database(
    name = "example_compdb",
    # OUTPUT_BASE is a dynamic value that will vary for each user workspace.
    # If you would like your build outputs to be the same across users, then
    # skip supplying this value, and substitute the default constant value
    # "__OUTPUT_BASE__" through an external tool like `sed` or `jq` (see
    # below shell commands for usage).
    output_base = OUTPUT_BASE,
    targets = [
        "//src/odroid_controller:controller",
    ],
)

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
        "@platforms//os:linux",
    ],
)
