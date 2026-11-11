from setuptools import find_packages, setup

package_name = 'gui_pista'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duarte Diamantino',
    maintainer_email='duarte.antonio.eu@gmail.com',
    description='GUI (Tkinter) para desenhar a pista, planear a trajetoria e publicar no ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_pista = gui_pista.app:main',
        ],
    },
)
