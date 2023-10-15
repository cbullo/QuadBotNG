# QuadBotNG

To compile for RCTimer platform:

`bazel build  @simplefoc//:simplefoc_lib --incompatible_enable_cc_toolchain_resolution=true --platforms=//:RCTimer --features=-gnu99`

To compile for Odroid-C4 platform:

`bazel build //src/odroid_controller:controller --incompatible_enable_cc_toolchain_resolution=true --platforms=//:Odroid-C4`

To compile everything and deploy to target robot:

`bazel run //src/odroid_controller/controller:deploy_controller --incompatible_enable_cc_toolchain_resolution=true --incompatible_use_platforms_repo_for_constraints=false --platforms=//:Odroid-C4 -- all`
