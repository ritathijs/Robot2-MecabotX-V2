from setuptools import setup

package_name = 'people_follower_ros2'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/visual_follower.launch.py']))
data_files.append(('share/' + package_name, ['launch/visual_follower.launch.py']))

data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='powrbv@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualtracker = people_follower_ros2.visualTracker:main',
            'visualfollow = people_follower_ros2.visualFollower:main',
    
        ],
    },
)
