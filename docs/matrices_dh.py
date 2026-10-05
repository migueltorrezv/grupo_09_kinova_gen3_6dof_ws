import os
import sys
import time

import numpy as np
import sympy as sp

DOCS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(DOCS, '..', 'src', 'grupo09_kinova_gen3_kinematics'))

from grupo09_kinova_gen3_kinematics.kinematics import forward_kinematics, position_jacobian

q = sp.symbols('q1:7', real=True)
d1, d2, d3, d4, d5, d6, a2 = sp.symbols('d_1 d_2 d_3 d_4 d_5 d_6 a_2', real=True)
pi = sp.pi

DH = [
    (q[0],          d1, 0,  pi / 2),
    (q[1] - pi / 2, d2, a2, pi),
    (q[2] - pi / 2, d3, 0,  pi / 2),
    (q[3],          d4, 0, -pi / 2),
    (q[4],          d5, 0,  pi / 2),
    (q[5] + pi,     d6, 0,  pi),
]

VALUES = {d1: -0.28481, d2: -0.005375, d3: -0.006375, d4: -0.31436,
          d5: -0.00035, d6: -0.167455, a2: 0.410}


def dh_matrix(theta, d, a, alpha):
    ct, st = sp.cos(theta), sp.sin(theta)
    ca, sa = sp.cos(alpha), sp.sin(alpha)
    return sp.Matrix([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,       ca,      d],
        [0,        0,        0,      1],
    ])


t0 = time.time()

A = [sp.simplify(dh_matrix(*row)) for row in DH]
print(f'Matrices A_i listas ({time.time() - t0:.1f} s)')

T = sp.diag(1, -1, -1, 1)
for Ai in A:
    T = T * Ai
T = T.applyfunc(lambda e: sp.trigsimp(sp.expand(e)))
print(f'T_base^6 simplificada ({time.time() - t0:.1f} s)')

p = T[:3, 3]
R = T[:3, :3]
J = p.jacobian(q).applyfunc(sp.trigsimp)
print(f'Jacobiano listo ({time.time() - t0:.1f} s)')

f_p = sp.lambdify(q, p.subs(VALUES), 'numpy')
f_R = sp.lambdify(q, R.subs(VALUES), 'numpy')
f_J = sp.lambdify(q, J.subs(VALUES), 'numpy')

rng = np.random.default_rng(0)
err_p = err_R = err_J = 0.0
for _ in range(5):
    qq = rng.uniform(-1.5, 1.5, 6)
    T_num = forward_kinematics(qq)
    err_p = max(err_p, np.abs(np.array(f_p(*qq), dtype=float).ravel() - T_num[:3, 3]).max())
    err_R = max(err_R, np.abs(np.array(f_R(*qq), dtype=float) - T_num[:3, :3]).max())
    err_J = max(err_J, np.abs(np.array(f_J(*qq), dtype=float) - position_jacobian(qq)).max())

print(f'Verificacion vs kinematics.py:  max err p = {err_p:.2e}   R = {err_R:.2e}   J = {err_J:.2e}')

c = sp.symbols('c_1:7')
s = sp.symbols('s_1:7')
SHORT = {}
for i in range(6):
    SHORT[sp.cos(q[i])] = c[i]
    SHORT[sp.sin(q[i])] = s[i]
SHORT[sp.cos(q[1] - q[2])] = sp.Symbol('c_{23}')
SHORT[sp.sin(q[1] - q[2])] = sp.Symbol('s_{23}')


def tex(expr):
    return sp.latex(expr.subs(SHORT))


out = ['% Generado por docs/matrices_dh.py  (c_i = cos q_i, s_i = sin q_i, c_23 = cos(q2 - q3), s_23 = sin(q2 - q3))', '']
for i, Ai in enumerate(A):
    out += ['\\begin{equation}', f'A_{{{i}}}^{{{i + 1}}} = {tex(Ai)}', '\\end{equation}', '']

for k, name in enumerate(['x', 'y', 'z']):
    out += ['\\begin{equation}', f'p_{name}(q) = {tex(p[k])}', '\\end{equation}', '']

for r in range(3):
    for col in range(3):
        out += ['\\begin{equation}', f'r_{{{r + 1}{col + 1}}} = {tex(R[r, col])}', '\\end{equation}', '']

for col in range(6):
    out += ['\\begin{equation}', f'J_{{v{col + 1}}} = {tex(J[:, col])}', '\\end{equation}', '']

with open(os.path.join(DOCS, 'matrices_dh.tex'), 'w') as f:
    f.write('\n'.join(out))

print(f'LaTeX guardado en {os.path.join(DOCS, "matrices_dh.tex")}  ({time.time() - t0:.1f} s)')
