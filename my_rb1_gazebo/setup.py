from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_rb1_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shreya',
    maintainer_email='shreyap.lanjewar@gmail.com',
    description='Package for spawning RB1 in Gazebo',
    license='MIT',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
