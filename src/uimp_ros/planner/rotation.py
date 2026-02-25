import numpy as np


def rotation_translation(x0, theta, h, w):
    """
    Compute the vehicle's polytope representation.

    Parameters:
    x0: 2x1 vector [x, y], representing the vehicle center coordinates
    theta: heading angle in radians
    h: vehicle total length
    w: vehicle width

    Returns:
    A: A matrix of the polytope representation
    b: b vector of the polytope representation
    """
    # rotation matrix
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])

    # small-angle approximation (commented out)
    # R = np.array([[1, -theta],
    #               [theta, 1]])

    # initialize b vector
    b = np.array([h / 2, w / 2, h / 2, w / 2])

    # model the vehicle's occupied area as a set of half-spaces (polytope)
    A = np.vstack([R.T, -R.T])  # A is the polytope matrix for the vehicle
    b = b + A @ x0  # b is the polytope offset vector for the vehicle

    return A, b