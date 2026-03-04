from setuptools import find_packages, setup

package_name = 'pose_persistence'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'amcl_pose_saver = pose_persistence.amcl_pose_saver:main',
        'initial_pose_restorer = pose_persistence.initial_pose_restorer:main',
    ],
   },
)
