load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

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
    urls = ["https://github.com/bazelregistry/yaml-cpp/archive/master.zip"],
)

# http_archive(
#     name = "simplefoc",
#     build_file = "@//:BUILD.simplefoc",
#     strip_prefix = "Arduino-FOC-2.2/src",
#     urls = ["https://github.com/simplefoc/Arduino-FOC/archive/refs/tags/v2.2.zip"],
# )

new_local_repository(
    name = "simplefoc",
    build_file = "@//:BUILD.simplefoc",
    path = "../Arduino-FOC/src",
)

http_archive(
    name = "arduino",
    build_file = "@//:BUILD.arduino",
    strip_prefix = "ArduinoCore-avr-1.8.3",
    urls = ["https://github.com/arduino/ArduinoCore-avr/archive/refs/tags/1.8.3.zip"],
)

load("@EmbeddedSystemsBuildScripts//Toolchains/Avr:avr.bzl", "avr_toolchain")

avr_toolchain()

load("//toolchain:toolchain.bzl", "register_all_toolchains")

register_all_toolchains()
