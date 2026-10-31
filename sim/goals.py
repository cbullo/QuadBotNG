"""Body-pose curricula, in metres and radians relative to the reset pose."""

AXIS_BODY_POSE_GOALS = (
    ("Crouch", (0, 0, -.008, 0, 0, 0)),
    ("Lean forward", (0, 0, 0, 0, .03, 0)),
    ("Lean back", (0, 0, 0, 0, -.03, 0)),
    ("Roll left", (0, 0, 0, .03, 0, 0)),
    ("Roll right", (0, 0, 0, -.03, 0, 0)),
    ("Shift forward", (.008, 0, 0, 0, 0, 0)),
    ("Shift back", (-.008, 0, 0, 0, 0, 0)),
    ("Shift left", (0, .005, 0, 0, 0, 0)),
    ("Shift right", (0, -.005, 0, 0, 0, 0)),
    ("Turn left", (0, 0, 0, 0, 0, .04)),
    ("Turn right", (0, 0, 0, 0, 0, -.04)),
)

# Larger, distinct goals that still fit the existing ±1 rad rotor action range.
# Preserve the original axis curriculum above for saved training configurations.
POSE_BODY_POSE_GOALS = (
    ("Turn left", (0, 0, 0, 0, 0, .15)),
    ("Turn right", (0, 0, 0, 0, 0, -.15)),
    ("Roll left", (0, 0, 0, .075, 0, 0)),
    ("Roll right", (0, 0, 0, -.075, 0, 0)),
    ("Shift forward", (.02, 0, 0, 0, 0, 0)),
    ("Shift back", (-.02, 0, 0, 0, 0, 0)),
    ("Shift left", (0, .014, 0, 0, 0, 0)),
    ("Shift right", (0, -.014, 0, 0, 0, 0)),
    ("Forward left twist", (.01, .01, 0, 0, 0, -.06)),
    ("Back right twist", (-.01, -.01, 0, 0, 0, .06)),
    ("Left shift and roll", (0, .012, 0, .04, 0, 0)),
    ("Right shift and roll", (0, -.012, 0, -.04, 0, 0)),
    ("Lean forward", (0, 0, 0, 0, .04, 0)),
    ("Lean back", (0, 0, 0, 0, -.04, 0)),
    ("Crouch", (0, 0, -.0085, 0, 0, 0)),
)

# Rotor radians, before the 9:1 reduction and three-motor differential.
# Leave headroom above the approximately 9.7 rad required for a 150 mm crouch.
WORKSPACE_ACTION_SCALE = 12.0
WORKSPACE_MIN_BASE_HEIGHT = .06

# Larger tilts/shifts start 50 mm below standing to leave extension headroom.
# Joint/tendon limits still come from the measured robot configuration.
BODY_POSE_GOALS = (
    ("Deep crouch", (0, 0, -.15, 0, 0, 0)),
    ("Stand tall", (0, 0, 0, 0, 0, 0)),
    ("Mid crouch", (0, 0, -.10, 0, 0, 0)),
    ("Shallow crouch", (0, 0, -.05, 0, 0, 0)),
    ("Lean forward", (0, 0, -.05, 0, .22, 0)),
    ("Lean back", (0, 0, -.05, 0, -.22, 0)),
    ("Roll left", (0, 0, -.05, .17, 0, 0)),
    ("Roll right", (0, 0, -.05, -.17, 0, 0)),
    ("Turn left", (0, 0, -.05, 0, 0, .22)),
    ("Turn right", (0, 0, -.05, 0, 0, -.22)),
    ("Shift forward", (.06, 0, -.05, 0, 0, 0)),
    ("Shift back", (-.06, 0, -.05, 0, 0, 0)),
    ("Shift left", (0, .03, -.05, 0, 0, 0)),
    ("Shift right", (0, -.03, -.05, 0, 0, 0)),
    ("Forward left twist", (.02, .01, -.05, .04, -.06, .06)),
    ("Back right twist", (-.02, -.01, -.05, -.04, .06, -.06)),
)

GOAL_SETS = {"axis": AXIS_BODY_POSE_GOALS, "poses": POSE_BODY_POSE_GOALS,
             "workspace": BODY_POSE_GOALS}
