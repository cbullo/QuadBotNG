"""Host motor and leg names used by the native inspector."""

LEGS = ("FR", "FL", "BR", "BL")
MOTORS = tuple(f"motor_{leg}{axis}" for leg in LEGS for axis in "IOZ")
