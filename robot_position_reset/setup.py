from setuptools import find_packages, setup

package_name = 'robot_position_reset'

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
    maintainer='wheeltec',
    maintainer_email='rita.thijs@kuleuven.be',
    description='Robot position reset node for charger alignment',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_position_reset = robot_position_reset.RobotPositionReset:main',
        ],
    },
)
