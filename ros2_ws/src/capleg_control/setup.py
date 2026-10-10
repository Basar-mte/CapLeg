from setuptools import find_packages, setup

package_name = 'capleg_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/controller.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Md. Abul Basar Roky',
    maintainer_email='m.a.basar.mte@gmail.com',
    description='Phase-based impedance control and treadmill hip-simulator logic for CapLeg',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'capleg_controller = capleg_control.controller_node:main',
            'gait_player = capleg_control.gait_player:main',
        ],
    },
)
