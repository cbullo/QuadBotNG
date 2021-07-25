def _impl(ctx):
    return [
        DefaultInfo(
            data_runfiles =
                ctx.attr.actual_binary[0][DefaultInfo].data_runfiles,
        ),
    ]

def _transition_impl(settings, attr):
    return {
        "//command_line_option:platforms": ["//:RCTimer"],
        "//command_line_option:features": ["-gnu99"],
    }

_comp_mode_transition = transition(
    implementation = _transition_impl,
    inputs = [],
    outputs = ["//command_line_option:platforms", "//command_line_option:features"],
)

transitioning_rule = rule(
    implementation = _impl,
    attrs = {
        "actual_binary": attr.label(cfg = _comp_mode_transition),
        "_allowlist_function_transition": attr.label(
            default = "@bazel_tools//tools/allowlists/function_transition_allowlist",
        ),
    },
)
