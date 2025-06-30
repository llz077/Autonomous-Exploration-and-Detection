from setuptools import find_packages, setup
from glob import glob

package_name = 'ri_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sutd',
    maintainer_email='sutd@todo.todo',
    description='Robotics Intelligence Challenge Package',
    license='BSD-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ri_challenge_node = ri_challenge.ri_challenge_node:main'
        ],
    },
)
