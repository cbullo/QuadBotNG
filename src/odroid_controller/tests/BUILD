load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "angle_tests",
    size = "small",
    srcs = ["angle_tests.cc"],
    deps = [
        "//src/odroid_controller:controller_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "board_tests",
    size = "small",
    srcs = ["board_tests.cc"],
    data = ["//src/odroid_controller/controller:config/robot_config.yaml"],
    deps = [
        "//src/odroid_controller:controller_lib",
        "//src/odroid_controller:emulated_board",
        "@com_google_googletest//:gtest_main",
    ],
)
