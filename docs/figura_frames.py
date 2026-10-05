import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DOCS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(DOCS, '..', 'src', 'grupo09_kinova_gen3_kinematics'))

from grupo09_kinova_gen3_kinematics.kinematics import forward_frames

frames = forward_frames(np.zeros(6))
origins = np.array([T[:3, 3] for T in frames])

display_shift = {3: 0.13, 5: 0.12}

fig = plt.figure(figsize=(8, 10))
ax = fig.add_subplot(projection='3d')

ax.plot(origins[:, 0], origins[:, 1], origins[:, 2],
        color='0.55', linewidth=7, alpha=0.4, solid_capstyle='round')

colors = ['tab:red', 'tab:green', 'tab:blue']
names = ['x', 'y', 'z']

for i, T in enumerate(frames):
    o_real = T[:3, 3]
    o = o_real + np.array([0.0, 0.0, display_shift.get(i, 0.0)])

    if i in display_shift:
        ax.plot(*zip(o_real, o), color='0.3', linestyle=':', linewidth=1.5)

    length = 0.11
    for k in range(3):
        d = T[:3, k] * length
        ax.quiver(*o, *d, color=colors[k], linewidth=2.2, arrow_length_ratio=0.25)
        ax.text(*(o + d * 1.45), f'${names[k]}_{i}$', color=colors[k], fontsize=11)

    ax.text(o[0], o[1] - 0.34, o[2] + 0.02, f'$O_{i}$', fontsize=12, fontweight='bold')

dims = [
    (0.0, 0.28481, '$d_1$ = 0.2848 m'),
    (0.28481, 0.69481, '$a_2$ = 0.410 m'),
    (0.69481, 1.00917, '$d_4$ = 0.3144 m'),
    (1.00917, 1.176625, '$d_6$ = 0.1675 m'),
]
xd = 0.32
for z0, z1, label in dims:
    ax.plot([xd, xd], [0, 0], [z0, z1], color='k', linewidth=1)
    ax.plot([xd - 0.02, xd + 0.02], [0, 0], [z0, z0], color='k', linewidth=1)
    ax.plot([xd - 0.02, xd + 0.02], [0, 0], [z1, z1], color='k', linewidth=1)
    ax.text(xd + 0.03, 0, (z0 + z1) / 2, label, fontsize=10)

ax.set_xlim(-0.45, 0.45)
ax.set_ylim(-0.45, 0.45)
ax.set_zlim(-0.10, 1.35)
ax.set_box_aspect((0.9, 0.9, 1.45))
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.view_init(elev=14, azim=-35)
ax.set_title('Asignación de frames DH — Kinova Gen3 6-DOF (q = 0)', fontsize=13)

fig.text(0.5, 0.03,
         'Frames 3 y 5 desplazados verticalmente solo para visualización (línea punteada).\n'
         'Sus orígenes reales coinciden con los de los frames 2 y 4 (diferencias < 7 mm).',
         ha='center', fontsize=9, style='italic')

fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(os.path.join(DOCS, 'figura_frames.png'), dpi=200)
fig.savefig(os.path.join(DOCS, 'figura_frames.pdf'))
print('Figura guardada en', DOCS)
