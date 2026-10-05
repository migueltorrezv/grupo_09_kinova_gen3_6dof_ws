import numpy as np
np.set_printoptions(precision=4, suppress=True)

def A(theta, d, a, alpha):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st*ca,  st*sa, a*ct],
                     [st,  ct*ca, -ct*sa, a*st],
                     [0,      sa,     ca,    d],
                     [0,       0,      0,    1]])

def fk(q):
    pi = np.pi
    dh = [(q[0],        -0.28481,   0.0,    pi/2),
          (q[1] - pi/2, -0.005375,  0.410,  pi),
          (q[2] - pi/2, -0.006375,  0.0,    pi/2),
          (q[3],        -0.31436,   0.0,   -pi/2),
          (q[4],        -0.00035,   0.0,    pi/2),
          (q[5] + pi,   -0.167455,  0.0,    pi)]
    T = np.diag([1.0, -1.0, -1.0, 1.0])  # T_base_0 = Rx(pi)
    for p in dh:
        T = T @ A(*p)
    return T

casos = {
    'q = 0': ([0, 0, 0, 0, 0, 0], [0.000, 0.001, 1.177]),
    'aleatoria': ([0.288398, 0.980672, 0.569512, -0.526531, 0.598576, -1.205743],
                  [0.415, -0.171, 0.961]),
}
for nombre, (q, p_ros) in casos.items():
    T = fk(q)
    err = np.linalg.norm(T[:3, 3] - np.array(p_ros))
    print(f'--- {nombre} ---')
    print('p_DH  =', T[:3, 3], '  p_ROS =', np.array(p_ros), f'  error = {err:.4f} m')
    print('R_DH =\n', T[:3, :3])
