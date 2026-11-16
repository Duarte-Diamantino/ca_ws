from setuptools import setup

package_name = 'ca_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duarte Diamantino',
    maintainer_email='duarte.antonio.eu@gmail.com',
    description='GUI da simulacao: mapa da pista, caminhos pela estrada, obstaculos, conduzir e afinar o MPC',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['ca_gui = ca_gui.app:main']},
)
