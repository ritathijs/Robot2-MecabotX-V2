from setuptools import setup
import os
from glob import glob

package_name = 'apriltag_sync'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='matthijs.mondelaers@student.kuleuven.be',
    description='Deze node start de ingebouwde apriltag_node op met de juiste parameters',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'apriltag_amcl_calibrator = apriltag_sync.apriltag_amcl_calibrator:main'
        ],
    },
)
