from glob import glob
import os
from setuptools import setup, find_packages

package_name = 'rover_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name, 'launch/'), glob('launch/*.launch.py')),
        (os.path.join('share/', package_name, 'config/'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miguel Ángel González Santamarta',
    maintainer_email='mgons@unileon.es',
    description='Rover teleop package',
    license='GPL-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard_node = rover_teleop.teleop_keyboard_node:main',
        ],
    },
)
