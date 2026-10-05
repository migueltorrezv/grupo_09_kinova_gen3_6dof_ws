import numpy as np

PI = np.pi

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']

DH_TABLE = [
    (0.0,     -0.28481,   0.000,  PI / 2),
    (-PI / 2, -0.005375,  0.410,  PI),
    (-PI / 2, -0.006375,  0.000,  PI / 2),
    (0.0,     -0.31436,   0.000, -PI / 2),
    (0.0,     -0.00035,   0.000,  PI / 2),
    (PI,      -0.167455,  0.000,  PI),
]

T_BASE_0 = np.diag([1.0, -1.0, -1.0, 1.0])

JOINT_LIMITS = np.array([
    [-PI,  PI],
    [-2.24, 2.24],
    [-2.57, 2.57],
    [-PI,  PI],
    [-2.09, 2.09],
    [-PI,  PI],
])


def dh_matrix(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,       ca,      d],
        [0.0,    0.0,      0.0,    1.0],
    ])


def forward_frames(q):
    frames = [T_BASE_0.copy()]
    T = T_BASE_0.copy()
    for qi, (offset, d, a, alpha) in zip(q, DH_TABLE):
        T = T @ dh_matrix(qi + offset, d, a, alpha)
        frames.append(T)
    return frames


def forward_kinematics(q):
    return forward_frames(q)[-1]


def position_jacobian(q):
    frames = forward_frames(q)
    o_n = frames[-1][:3, 3]
    J = np.zeros((3, 6))
    for i in range(6):
        z = frames[i][:3, 2]
        o = frames[i][:3, 3]
        J[:, i] = np.cross(z, o_n - o)
    return J


def rotation_to_quaternion(R):
    tr = np.trace(R)
    if tr > 0.0:
        s = 2.0 * np.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    quat = np.array([x, y, z, w])
    return quat / np.linalg.norm(quat)


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)

    q = np.array([0.288398, 0.980672, 0.569512,
                  -0.526531, 0.598576, -1.205743])
    T = forward_kinematics(q)
    R = T[:3, :3]
    print('p =', T[:3, 3])
    print('R^T R = I ?', np.allclose(R.T @ R, np.eye(3)))
    print('det(R) =', round(np.linalg.det(R), 6))
    print('quat (x,y,z,w) =', rotation_to_quaternion(R))

    J = position_jacobian(q)
    h = 1e-6
    J_num = np.zeros((3, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = h
        J_num[:, i] = (forward_kinematics(q + dq)[:3, 3]
                       - forward_kinematics(q - dq)[:3, 3]) / (2 * h)
    print('max |J - J_num| =', np.abs(J - J_num).max())
