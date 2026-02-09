from setuptools import setup

package_name = 'quiz_bt_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'python-socketio'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='rthijs@yahoo.com',
    description='Combined ROS 2 node for quiz server and robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quiz_bt_node = quiz_bt_node.quiz_bt_node:main'
        ],
    },
)

