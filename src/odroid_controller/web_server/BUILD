cc_library(
    name = "web_server",
    srcs = [
        "server.cc",
    ],
    hdrs = ["server.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//:libwebsockets",
        "//src/odroid_controller:controller_lib",
    ],
)
