from setuptools import setup
package_name = 'vision_obstacle_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # ← 只安装 vision_obstacle_detection 子目录
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            # 入口 = 包名.文件名:main
            'vision_obstacle_detection_node = vision_obstacle_detection.vision_obstacle_detection_node:main',
        ],
    },
)
