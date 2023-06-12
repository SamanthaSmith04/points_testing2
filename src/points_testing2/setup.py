#! /usr/bin/env python3

from setuptools import setup

package_name = 'points_testing2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samantha Smith',
    maintainer_email='smith.15485@osu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rdp_algorithm = points_testing2.rdp_algorithm:main',
            'corrector_functions = points_testing2.corrector_functions:main',
            'downsample = points_testing2.downsample:main',
            'test = points_testing2.test_client:main'
        ],
    },
)
