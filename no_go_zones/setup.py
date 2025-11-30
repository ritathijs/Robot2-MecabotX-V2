from setuptools import setup
import os
from glob import glob

package_name = 'no_go_zones'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    
    # Deze 'data_files' sectie is cruciaal
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Zorgt ervoor dat de /map map wordt meegenomen
        (os.path.join('share', package_name, 'map'), glob('map/*.*')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='matthijs.mondelaers@student.kuleuven.be',
    description='Publiceert virtuele obstakels op basis van kaart-pixels',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Definieert het 'ros2 run no_go_zones no_go_zones' commando
            'no_go_zones = no_go_zones.no_go_zones:main',
        ],
    },
)
