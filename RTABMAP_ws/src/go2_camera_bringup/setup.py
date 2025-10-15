from setuptools import setup
from glob import glob

package_name = 'go2_camera_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Bringup RealSense + Odom->TF',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_to_tf = go2_camera_bringup.odom_to_tf:main',
            'odom_time_sync = go2_camera_bringup.odom_time_sync:main',
        ],
    },
)
