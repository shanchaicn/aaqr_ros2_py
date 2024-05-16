from setuptools import find_packages, setup

package_name = 'aaqr_ros2_py'

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
    maintainer='shan',
    maintainer_email='shan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_poses_node = aaqr_ros2_py.save_poses_node:main',
            'follow_waypoints_from_yaml = aaqr_ros2_py.follow_waypoints_from_yaml:main',
            'framelistener = aaqr_ros2_py.framelistener:main',
            'waypoints_move_node = aaqr_ros2_py.waypoints_move_ndoe:main'
        ],
    },
)
