from setuptools import find_packages, setup

package_name = 'aptag_detection'

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
    maintainer='agv',
    maintainer_email='agv@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_detection_node = aptag_detection.tag_detection_node:main',
            'tf_broadcaster_node = aptag_detection.tf_broadcaster_node:main',
            'static_tf_broadcaster = aptag_detection.static_tf_broadcaster:main',
            'pose_to_marker_node = aptag_detection.pose_to_marker_node:main',
        ],
    },
)
