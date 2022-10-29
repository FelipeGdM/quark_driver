from setuptools import setup
from glob import glob
import os

package_name = 'cone_detection_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'cone_detection/utils', 'cone_detection/models'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tocoquinho',
    maintainer_email='giovannicmorales@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_sub = cone_detection.camera_sub_node:main',
        ],
    },
)
