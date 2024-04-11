import os
from glob import glob
from setuptools import setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include rviz files
        (os.path.join('share', package_name, 'config'), glob('config/*config.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonathan',
    maintainer_email='jonathan.a.diller@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_gap = robot_control.reactive_node:main',
        ],
    },
)
