from setuptools import setup

package_name = 'gpio_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'Jetson.GPIO'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@example.com',
    description='ROS2 node to read GPIO on Jetson Orin Nano',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'read_gpio_node = gpio_reader.read_gpio:main',
        ],
    },
)

