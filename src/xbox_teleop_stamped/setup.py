from setuptools import find_packages, setup

package_name = 'xbox_teleop_stamped'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/xbox_teleop_stamped.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dingo',
    maintainer_email='dingo@private.com',
    description='Xbox controller teleop with TwistStamped support for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_teleop_node = xbox_teleop_stamped.xbox_teleop_node:main',
        ],
    },
)
