import numpy as np


def cartesian_to_polar(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return rho, phi


def polar_to_cartesian(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y


def normalize_coordinates(x, y, angle):
    r, theta = cartesian_to_polar(x, y)
    theta = theta - angle
    x, y = polar_to_cartesian(r, theta)
    return x, y
