from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agv_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include package.xml
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Changye',
    maintainer_email='c7ma@uwaterloo.ca',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # make sure add the executable file name here
            'agv_controller = agv_controller.agv_controller:main',
            
        ],
    },
)
