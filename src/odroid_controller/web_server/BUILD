load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "web_server",
    srcs = [
        "server.cc",
    ],
    hdrs = ["server.h"],
    # Incomplete; retained for rework, excluded from wildcard builds.
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        "//:libwebsockets",
        "//src/odroid_controller:controller_lib",
    ],
)
