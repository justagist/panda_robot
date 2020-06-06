import quaternion
import numpy as np

def quat_convert(q):
    if isinstance(q, np.quaternion):
        return quaternion.as_float_array(q)
    else:
        return q


def quat_conj(q):
    return np.hstack([q[0], -q[1:4]])


def quat_mult(q1, q2):
    s1 = q1[0]
    s2 = q2[0]
    v1 = q1[1:4]
    v2 = q2[1:4]
    return np.hstack([s1*s2 - np.dot(v1, v2), s1*v2 + s2*v1 + np.cross(v1, v2)])


def compute_log(q):
    v = q[1:4]
    norm_v = np.linalg.norm(v)
    if norm_v == 0:
        return np.zeros(3)
    else:
        return np.arccos(q[0])*v/norm_v

def compute_omg(q1, q2):

    return 2.*compute_log(quat_mult(quat_convert(q1), quat_conj(quat_convert(q2))))
