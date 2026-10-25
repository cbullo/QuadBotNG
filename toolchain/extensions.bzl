"""Adapt the existing AVR repository generator to Bzlmod."""

load("@EmbeddedSystemsBuildScripts//Toolchains/Avr:avr.bzl", "create_avr_toolchain")
load("@EmbeddedSystemsBuildScripts//Toolchains/Avr:platforms/platform_list.bzl", "platforms")

def _avr_impl(_ctx):
    # avr_toolchain() also calls native.register_toolchains, which cannot run
    # in an extension. Toolchain registration belongs in MODULE.bazel.
    create_avr_toolchain(name = "AvrToolchain", mcu_list = platforms)

avr = module_extension(implementation = _avr_impl)
