"""Nearest-segment projection shared by standalone and integrated experiments."""
import math
import numpy as np


def project(xy, starts, vectors, lengths2):
    delta = xy - starts
    t = np.clip(np.sum(delta*vectors, axis=1)/lengths2, 0, 1)
    offsets = delta - t[:, None]*vectors
    index = int(np.argmin(np.sum(offsets**2, axis=1)))
    vx, vy = vectors[index]
    dx, dy = offsets[index]
    signed_distance = math.copysign(math.hypot(dx, dy), vx*dy-vy*dx)
    return index, t[index], signed_distance
