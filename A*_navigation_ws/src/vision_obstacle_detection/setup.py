from setuptools import find_packages, setup
from glob import glob
import os
setup(
    name="vision_obstacle_detection",
    version='0.0.0',    
    packages=find_packages(include=['src', 'src.*']),
    
    data_files=[
        # 安装消息文件（如果存在）
        (os.path.join('share', 'src', 'msg'), glob('msg/*.msg')),
        # 安装 package.xml
        (os.path.join('share', 'src'), ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/vision_obstacle_detection']),
        ('share/vision_obstacle_detection', ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zender_zhang',
    maintainer_email='zender_zhang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_obstacle_detection_node = src.vision_obstacle_detection_node:main',
        ],
    },
)
