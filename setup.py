from glob import glob
from setuptools import setup

package_name = 'slam_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'resource/slam.rviz',
         'resource/euroc_stereo.yaml', 'resource/orb_vocab.fbow']),
        ('share/' + package_name, glob('slam_launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swallak',
    maintainer_email='sw.allak@gmail.com',
    description='Launch files for SLAM application',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
