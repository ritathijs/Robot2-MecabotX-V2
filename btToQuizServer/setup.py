from setuptools import find_packages, setup

package_name = 'btToQuizServer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-socketio'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='rthijs@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'punisher_node = btToQuizServer.punisher_node:main',
        ],
    },
)
