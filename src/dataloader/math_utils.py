import warnings

import numpy as np
from numba import jit


def hat(v):
    v = np.squeeze(v)
    R = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return R


@jit(nopython=True, parallel=False, cache=True)
def rot_2vec(a, b):
    assert a.shape == (3, 1)
    assert b.shape == (3, 1)

    def hat(v):
        v = v.flatten()
        R = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return R

    a_n = np.linalg.norm(a)
    b_n = np.linalg.norm(b)
    a_hat = a / a_n
    b_hat = b / b_n
    omega = np.cross(a_hat.T, b_hat.T).T
    c = 1.0 / (1 + np.dot(a_hat.T, b_hat))
    R_ba = np.eye(3) + hat(omega) + c * hat(omega) @ hat(omega)
    return R_ba


@jit(nopython=True, parallel=False, cache=True)
def mat_exp(omega):
    if len(omega) != 3:
        raise ValueError("tangent vector must have length 3")

    def hat(v):
        v = v.flatten()
        R = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return R

    angle = np.linalg.norm(omega)

    # Near phi==0, use first order Taylor expansion
    if angle < 1e-10:
        return np.identity(3) + hat(omega)

    axis = omega / angle
    s = np.sin(angle)
    c = np.cos(angle)

    return c * np.identity(3) + (1 - c) * np.outer(axis, axis) + s * hat(axis)


mat_exp_vec = np.vectorize(mat_exp, signature="(3)->(3,3)")


def mat_log(R):
    q = compute_q_from_matrix(R)
    w = q[3]
    vec = q[0:3]
    n = np.linalg.norm(vec)
    epsilon = 1e-7

    if n < epsilon:
        w2 = w * w
        n2 = n * n
        atn = 2.0 / w - (2.0 * n2) / (w * w2)
    else:
        if np.absolute(w) < epsilon:
            if w > 0:
                atn = np.pi / n
            else:
                atn = -np.pi / n
        else:
            atn = 2.0 * np.arctan(n / w) / n
    tangent = atn * vec
    return tangent


def mat_log_vec(R):
    """
    Args:
        R [n x 3 x 3]
    """

    q = compute_q_from_matrix(R)
    w = q[:, 3]
    vec = q[:, 0:3]
    n = np.linalg.norm(vec, axis=1)
    epsilon = 1e-7

    mask = n < epsilon
    atn_small = 2.0 / w - (2.0 * n * n) / (w * w * w)

    mask2 = np.absolute(w) < epsilon
    atn_normal_small = np.sign(w) * np.pi / n
    atn_normal_normal = 2.0 * np.arctan(n / w) / n

    atn = mask2 * atn_normal_small + (1 - mask2) * atn_normal_normal
    atn = mask * atn_small + (1 - mask2) * atn

    tangent = atn[0, np.newaxis] * vec
    return tangent


""" right jacobian for exp operation on SO(3) """


@jit(nopython=True, parallel=False, cache=True)
def Jr_exp(phi):
    def hat(v):
        v = v.flatten()
        R = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return R

    theta = np.linalg.norm(phi)
    if theta < 1e-3:
        J = np.eye(3) - 0.5 * hat(phi) + 1.0 / 6.0 * (hat(phi) @ hat(phi))
    else:
        J = (
            np.eye(3)
            - (1 - np.cos(theta)) / np.power(theta, 2.0) * hat(phi)
            + (theta - np.sin(theta)) / np.power(theta, 3.0) * (hat(phi) @ hat(phi))
        )
    return J


def Jr_log(phi):
    """ right jacobian for log operation on SO(3) """
    theta = np.linalg.norm(phi)
    if theta < 1e-3:
        J = np.eye(3) + 0.5 * hat(phi)
    else:
        J = (
            np.eye(3)
            + 0.5 * hat(phi)
            + (
                1 / np.power(theta, 2.0)
                + (1 + np.cos(theta)) / (2 * theta * np.sin(theta))
            )
            * hat(phi)
            * hat(phi)
        )
    return J


def unwrap_rpy(rpys):
    diff = rpys[1:, :] - rpys[0:-1, :]
    uw_rpys = np.zeros(rpys.shape)
    uw_rpys[0, :] = rpys[0, :]
    diff[diff > 300] = diff[diff > 300] - 360
    diff[diff < -300] = diff[diff < -300] + 360
    uw_rpys[1:, :] = uw_rpys[0, :] + np.cumsum(diff, axis=0)
    return uw_rpys


def wrap_rpy(uw_rpys):
    rpys = uw_rpys
    while rpys.min() < -180:
        rpys[rpys < -180] = rpys[rpys < -180] + 360
    while rpys.max() >= 180:
        rpys[rpys >= 180] = rpys[rpys >= 180] - 360
    return rpys


def compute_q_from_matrix(matrix):
    is_single = False
    matrix = np.asarray(matrix, dtype=float)

    if matrix.ndim not in [2, 3] or matrix.shape[-2:] != (3, 3):
        raise ValueError(
            "Expected `matrix` to have shape (3, 3) or "
            "(N, 3, 3), got {}".format(matrix.shape)
        )

    # If a single matrix is given, convert it to 3D 1 x 3 x 3 matrix but
    # set self._single to True so that we can return appropriate objects in
    # the `to_...` methods
    if matrix.shape == (3, 3):
        matrix = matrix.reshape((1, 3, 3))
        is_single = True

    num_rotations = matrix.shape[0]

    decision_matrix = np.empty((num_rotations, 4))
    decision_matrix[:, :3] = matrix.diagonal(axis1=1, axis2=2)
    decision_matrix[:, -1] = decision_matrix[:, :3].sum(axis=1)
    choices = decision_matrix.argmax(axis=1)

    quat = np.empty((num_rotations, 4))

    ind = np.nonzero(choices != 3)[0]
    i = choices[ind]
    j = (i + 1) % 3
    k = (j + 1) % 3

    quat[ind, i] = 1 - decision_matrix[ind, -1] + 2 * matrix[ind, i, i]
    quat[ind, j] = matrix[ind, j, i] + matrix[ind, i, j]
    quat[ind, k] = matrix[ind, k, i] + matrix[ind, i, k]
    quat[ind, 3] = matrix[ind, k, j] - matrix[ind, j, k]

    ind = np.nonzero(choices == 3)[0]
    quat[ind, 0] = matrix[ind, 2, 1] - matrix[ind, 1, 2]
    quat[ind, 1] = matrix[ind, 0, 2] - matrix[ind, 2, 0]
    quat[ind, 2] = matrix[ind, 1, 0] - matrix[ind, 0, 1]
    quat[ind, 3] = 1 + decision_matrix[ind, -1]

    quat /= np.linalg.norm(quat, axis=1)[:, None]

    if is_single:
        return quat[0]
    else:
        return quat

