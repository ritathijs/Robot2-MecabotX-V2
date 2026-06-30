from setuptools import setup

package_name = 'autocharge_batcheck'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'python-socketio'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='rthijs@yahoo.com',
    description='Code om naar spanningsniveaus te kijken',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autocharge_batcheck = autocharge_batcheck.autocharge_batcheck:main'
        ],
    },
)

