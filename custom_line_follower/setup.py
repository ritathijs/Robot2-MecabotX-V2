from setuptools import setup
import os
from glob import glob

package_name = 'custom_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Dit zorgt dat je launch files worden geïnstalleerd
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Custom Line Follower variant',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Commando = PackageMap.BestandsNaam:Functie
            'line_follow_node = custom_line_follower.line_follower_node:main',
        ],
    },
)
