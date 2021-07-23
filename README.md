# QuadBotNG

To compile for RCTimer platform:

`bazel build  @simplefoc//:simplefoc_lib --incompatible_enable_cc_toolchain_resolution=true --platforms=//:RCTimer --features=-gnu99`