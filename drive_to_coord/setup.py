from setuptools import setup

package_name = 'drive_to_coord'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='matthijs.mondelaers@student.kuleuven.be',
    description='Vraagt coördinaten en stuurt de robot ernaartoe.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drive_to_coord = drive_to_coord.drive_to_coord:main',
        ],
    },
)

