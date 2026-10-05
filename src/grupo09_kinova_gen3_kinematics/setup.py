from setuptools import find_packages, setup

package_name = 'grupo09_kinova_gen3_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miguel Angel Torrez',
    maintainer_email='miguel.torrez.v@ucb.edu.bo',
    description='Cinemática directa e inversa del Kinova Gen3 6-DOF (Grupo 09, IMT-342)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_node = grupo09_kinova_gen3_kinematics.ik_node:main',
            'fk_node = grupo09_kinova_gen3_kinematics.fk_node:main',
        ],
    },
)
