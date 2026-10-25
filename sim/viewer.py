"""Run MuJoCo's standard desktop viewer using Bazel-managed dependencies."""

import os
import runpy


if __name__ == "__main__":
    # Resolve --mjcf paths relative to the caller, not Bazel's runfiles tree.
    if "BUILD_WORKING_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])
    runpy.run_module("mujoco.viewer", run_name="__main__")
