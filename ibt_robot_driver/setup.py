from setuptools import setup
from glob import glob
import os

package_name = 'ibt_robot_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=[
        'setuptools',
        'gbp==0.0.2'
    ],
    dependency_links=[
        'https://downloads.glowbuzzer.com/releases/gbp/index.html'
    ],
    zip_safe=True,
    maintainer='Mattia Dei Rossi',
    maintainer_email='mattia.deirossi@innobotics.it',
    description='ROS2 driver for robofox',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ibt_robot_driver = ibt_robot_driver.main:main'
        ],
    },
)