import numpy as np


def rotx(theta):
    rot = np.array([[1.0, 0, 0], [0, 1, 0], [0, 0, 1]])
    rot[1, 1] = np.cos(theta)
    rot[2, 1] = np.sin(theta)
    rot[1, 2] = -np.sin(theta)
    rot[2, 2] = np.cos(theta)
    return rot

def roty(theta):
    rot = np.array([[1.0, 0, 0], [0, 1, 0], [0, 0, 1]])
    rot[2, 2] = np.cos(theta)
    rot[0, 2] = np.sin(theta)
    rot[2, 0] = -np.sin(theta)
    rot[0, 0] = np.cos(theta)
    return rot

def rotz(theta):
    rot = np.array([[1.0, 0, 0], [0, 1, 0], [0, 0, 1]])
    rot[0, 0] = np.cos(theta)
    rot[1, 0] = np.sin(theta)
    rot[0, 1] = -np.sin(theta)
    rot[1, 1] = np.cos(theta)
    return rot

