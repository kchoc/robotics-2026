def mod_angle(angle: float) -> float:
    """Normalize an angle to the range [-π, π]."""
    from numpy import pi
    return (angle + pi) % (2 * pi) - pi