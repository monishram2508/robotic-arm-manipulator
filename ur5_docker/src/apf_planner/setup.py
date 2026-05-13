from setuptools import find_packages, setup

package_name = 'apf_planner'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'pointcloud_robot_filter = apf_planner.pointcloud_robot_filter:main',
        'apf_local_planner_node = apf_planner.apf_local_planner_node:main',
        'ur5_pick_place_controller = apf_planner.ur5_pick_place_controller:main',
    ],
},
)
