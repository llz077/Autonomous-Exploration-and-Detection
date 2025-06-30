from setuptools import setup
from glob import glob
 
package_name = 'alert_player'
 
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/media', glob('media/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Achala Athukorala',
    maintainer_email='achala_chathuranga@sutd.edu.sg',
    description='Audio Alert Player ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alert_player = alert_player.alert_player:main'
        ],
    },
)
