from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kuka_waam_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='d',
    maintainer_email='dismaskarimidissy64@gmail.com',
    description='WAAM control package for KUKA robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gcode_transpiler.py = kuka_waam_control.gcode_transpiler:main',
            'krl_executor_node.py = kuka_waam_control.krl_executor_node:main',
            'waam_state_machine.py = kuka_waam_control.waam_state_machine:main',
            'safety_monitor.py = kuka_waam_control.safety_monitor:main',
        ],
    },
)

