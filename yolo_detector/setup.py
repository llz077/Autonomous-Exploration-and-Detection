from setuptools import setup
from glob import glob
 
package_name = 'yolo_detector'
 
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='achala',
    maintainer_email='achala@todo.todo',
    description='YOLO Object Detector ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_detector.yolo_node:main'
        ],
    },
)
