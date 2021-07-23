load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Change master to the git tag you want.
http_archive(
    name = "com_grail_bazel_compdb",
    strip_prefix = "bazel-compilation-database-master",
    urls = ["https://github.com/grailbio/bazel-compilation-database/archive/master.tar.gz"],
)

http_archive(
    name = "EmbeddedSystemsBuildScripts",
    strip_prefix = "EmbeddedSystemsBuildScripts-1.0.2",
    urls = ["https://github.com/es-ude/EmbeddedSystemsBuildScripts/archive/v1.0.2.tar.gz"]
)

http_archive(
    name = "yaml-cpp",
    strip_prefix = "yaml-cpp-master",
    urls = ["https://github.com/bazelregistry/yaml-cpp/archive/master.zip"],
)

http_archive(
    name = "simplefoc",
    urls = ["https://github.com/simplefoc/Arduino-FOC/archive/refs/tags/v2.1.1.zip"],
    build_file = "@//:BUILD.simplefoc",
    strip_prefix = "Arduino-FOC-2.1.1/src"
)

http_archive(
    name = "arduino",
    strip_prefix = "ArduinoCore-avr-1.8.3",
    urls = ["https://github.com/arduino/ArduinoCore-avr/archive/refs/tags/1.8.3.zip"],
    build_file = "@//:BUILD.arduino"
)

load("@EmbeddedSystemsBuildScripts//Toolchains/Avr:avr.bzl", "avr_toolchain")

avr_toolchain()

