from setuptools import setup

package_name = 'bumper'

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
    description='Leest de GPIO uit en handeld de bewegingen uit',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bumper = bumper.bumper:main',
        ],
    },
)

