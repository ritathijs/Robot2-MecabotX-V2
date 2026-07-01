from setuptools import setup

package_name = 'april_tabloo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'python-socketio'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='rthijs@yahoo.com',
    description='Apriltag versie tabloo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'april_tabloo = april_tabloo.april_tabloo:main'
        ],
    },
)

