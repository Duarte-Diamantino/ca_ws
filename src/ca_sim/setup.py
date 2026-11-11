from glob import glob
import os

from setuptools import setup

package_name = 'ca_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duarte Diamantino',
    maintainer_email='duarte.antonio.eu@gmail.com',
    description='Carro simulado (cinematico ou Gazebo) e testes em circuito fechado do MPC/DWA.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematic_sim = ca_sim.kinematic_sim:main',
            'tracking_monitor = ca_sim.tracking_monitor:main',
            'vesc_to_gz = ca_sim.vesc_to_gz:main',
            'gz_spawner = ca_sim.gz_spawner:main',
        ],
    },
)
