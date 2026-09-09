from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'urxp_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*')),
        (os.path.join('share', package_name, 'models', 'target_cube'),
            glob('models/target_cube/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Asraful Hasan',
    maintainer_email='asrafulhasan1996@gmail.com',
    description='Standalone RealSense + xArm Lite6 pick-and-place of a colored cube',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cube_detector = urxp_pick_place.cube_detector:main',
            'pose_filter = urxp_pick_place.pose_filter:main',
            'gripper_node = urxp_pick_place.gripper_node:main',
            'gripper_sim_node = urxp_pick_place.gripper_sim_node:main',
            'pick_place_server = urxp_pick_place.pick_place_server:main',
            'table_publisher = urxp_pick_place.table_publisher:main',
        ],
    },
)
