from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'urxp_web_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'static'),
            glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Asraful Hasan',
    maintainer_email='asrafulhasan1996@gmail.com',
    description='Local web dashboard for URXP pick-and-place',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_ui_server = urxp_web_ui.server:main',
        ],
    },
)
