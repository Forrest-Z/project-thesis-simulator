#!/usr/bin/env python
import time

import numpy as np

class Controller(object):
    def __init__(self):
        pass

    def update(self, vobj):
        pass

    def draw(self):
        pass

    def visualize(self, axes, t, n):
        pass

def normalize_angle(angle, angle_ref):
    """
    Makes 'angle' compatible with 'angle_ref' such that the numerical
    difference is at most PI
    """
    if angle_ref == np.Inf:
        return angle

    # Get angle within 2*PI of angle_ref
    diff = angle_ref - angle
    if diff > 0:
        new_angle = angle + (diff - np.fmod(diff, 2*np.pi))
    else:
        new_angle = angle + (diff + np.fmod(-diff, 2*np.pi))

    # Make sure angle is on the closest side of angle_ref
    diff = angle_ref - new_angle
    if diff > np.pi:
        new_angle += 2*np.pi
    elif diff < -np.pi:
        new_angle -= 2*np.pi

    return new_angle


def int_circles(x1, y1, r1, x2, y2, r2):
    """Tests for intersection between two circles.

    Returns:
        int = True on intersection
        coords = coordinates of intersection (numpy.array)

    http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/
    """

    # Distance between circle centers
    d = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    if (d > r1 + r2) or (d < np.abs(r1-r2)):
        # No solution
        return False, []

    a = (r1**2 - r2**2 + d**2 ) / (2*d)

    xp = x1 + a*(x2 - x1)/d
    yp = y1 + a*(y2 - y1)/d

    if (d == r1 + r2):
        # One solution
        coords = np.array([xp, yp])
        return True, coords
    else:
        # Two solutions
        h = np.sqrt(r1**2 - a**2);
        coords = np.array([[xp+h*(y2-y1)/d, yp-h*(x2-x1)/d],
                           [xp-h*(y2-y1)/d, yp+h*(x2-x1)/d]])
    return True, coords
