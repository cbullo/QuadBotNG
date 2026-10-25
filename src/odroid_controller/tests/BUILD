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
