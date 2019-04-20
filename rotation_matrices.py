import numpy as np

def rotx(theta):
    rot = np.array([[1 0 0], [0 1 0], [0 0 1]])
    rot(2, 2) = np.cos(theta)
    rot(2, 3) = -np.sin(theta)
    rot(3, 2) = np.sin(theta)
    rot(3, 3) = np.cos(theta)
    return rot