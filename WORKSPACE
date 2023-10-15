load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "bazel_skylib",
    sha256 = "b8a1527901774180afc798aeb28c4634bdccf19c4d98e7bdd1ce79d1fe9aaad7",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-skylib/releases/download/1.4.1/bazel-skylib-1.4.1.tar.gz",
        "https://github.com/bazelbuild/bazel-skylib/releases/download/1.4.1/bazel-skylib-1.4.1.tar.gz",
    ],
)

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

http_archive(
    name = "com_grail_bazel_compdb",
    strip_prefix = "bazel-compilation-database-0.5.2",
    urls = ["https://github.com/grailbio/bazel-compilation-database/archive/0.5.2.tar.gz"],
)

load("@com_grail_bazel_compdb//:deps.bzl", "bazel_compdb_deps")

bazel_compdb_deps()

http_archive(
    name = "EmbeddedSystemsBuildScripts",
    strip_prefix = "EmbeddedSystemsBuildScripts-1.0.2",
    urls = ["https://github.com/es-ude/EmbeddedSystemsBuildScripts/archive/v1.0.2.tar.gz"],
)

http_archive(
    name = "yaml-cpp",
    strip_prefix = "yaml-cpp-master",
    urls = ["https://github.com/jbeder/yaml-cpp/archive/master.zip"],
)

http_archive(
    name = "fmt",
    patch_cmds = [
        "mv support/bazel/.bazelrc .bazelrc",
        "mv support/bazel/.bazelversion .bazelversion",
        "mv support/bazel/BUILD.bazel BUILD.bazel",
        "mv support/bazel/WORKSPACE.bazel WORKSPACE.bazel",
    ],
    strip_prefix = "fmt-8.1.1",
    urls = ["https://github.com/fmtlib/fmt/archive/refs/tags/8.1.1.tar.gz"],
)

http_archive(
    name = "simplefoc",
    build_file = "@//:BUILD.simplefoc",
    strip_prefix = "Arduino-FOC-master/src",
    urls = ["https://github.com/cbullo/Arduino-FOC/archive/refs/heads/master.zip"],
)

# new_local_repository(
#     name = "simplefoc",
#     build_file = "@//:BUILD.simplefoc",
#     path = "../Arduino-FOC/src",
# )

http_archive(
    name = "arduino",
    build_file = "@//:BUILD.arduino",
    strip_prefix = "ArduinoCore-avr-1.8.5",
    urls = ["https://github.com/arduino/ArduinoCore-avr/archive/refs/tags/1.8.5.zip"],
)

_ALL_CONTENT = """
filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)
"""

http_archive(
    name = "libwebsockets",
    build_file_content = _ALL_CONTENT,
    strip_prefix = "libwebsockets-4.3.2",
    urls = ["https://github.com/warmcat/libwebsockets/archive/refs/tags/v4.3.2.zip"],
)

load("@EmbeddedSystemsBuildScripts//Toolchains/Avr:avr.bzl", "avr_toolchain")

avr_toolchain()

load("//toolchain:toolchain.bzl", "register_all_toolchains")

register_all_toolchains()

http_archive(
    name = "rules_foreign_cc",
    sha256 = "2a4d07cd64b0719b39a7c12218a3e507672b82a97b98c6a89d38565894cf7c51",
    strip_prefix = "rules_foreign_cc-0.9.0",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/refs/tags/0.9.0.tar.gz",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

# This sets up some common toolchains for building targets. For more details, please see
# https://bazelbuild.github.io/rules_foreign_cc/0.9.0/flatten.html#rules_foreign_cc_dependencies
rules_foreign_cc_dependencies()
