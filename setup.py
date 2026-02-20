from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'python_parameters'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gonazza',
    maintainer_email='jacopo.soncini@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazy_node = python_parameters.crazy_node:main',
            'leader = python_parameters.leader:main',
            'obstacles = python_parameters.obstacles:main'
            
        ],
    },
)
