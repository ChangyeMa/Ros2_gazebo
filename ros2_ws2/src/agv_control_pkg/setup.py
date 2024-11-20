from setuptools import find_packages, setup

package_name = 'agv_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='changye',
    maintainer_email='c7ma@uwaterloo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agv_controller= agv_control_pkg.agv_controller:main',
            'tag_detection= agv_control_pkg.tag_detection_node:main',
            'line_detection= agv_control_pkg.line_detection_node:main',
            'tf_broadcaster= agv_control_pkg.tf_broadcaster_node:main',
            'tag_detection_old= agv_control_pkg.tag_detection_node_old:main',
        ],
    },
)
