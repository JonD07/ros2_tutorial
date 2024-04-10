from setuptools import setup

package_name = 'basic_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Diller',
    maintainer_email='jdiller@mines.edu',
    description='Basic ROS2 Tutorial Code',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = basic_tutorial.publisher_member_function:main',
            'listener = basic_tutorial.subscriber_member_function:main',
            'service = basic_tutorial.service_member_function:main',
            'client = basic_tutorial.client_member_function:main',
        ],
    },
)
