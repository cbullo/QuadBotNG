# QuadBotNG

To compile for RCTimer platform:

`bazel build  @simplefoc//:simplefoc_lib --incompatible_enable_cc_toolchain_resolution=true --platforms=//:RCTimer --features=-gnu99`

To compiler for Odroid-C4 platform:

`bazel build //src/odroid_controller:controller --incompatible_enable_cc_toolchain_resolution=true --platforms=//:Odroid-C4`