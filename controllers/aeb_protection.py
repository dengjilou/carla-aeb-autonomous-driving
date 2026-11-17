"""Distance protection level; positive closing speed means approaching (m/s)."""


def distance_level(distance, closing, closing_guard=False):
    # Experimental 5 s constant-relative-speed window; TTC state thresholds stay separate.
    if closing_guard and distance>0 and (closing<=.1 or distance/closing>5.):
        return 0
    return 3 if distance<10 else 2 if distance<15 else 0
