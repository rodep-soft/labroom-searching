from setuptools import setup
import os
from glob import glob

package_name = 'bringup'

setup(
    name='bringup',
    version='0.0.0',
    packages=package_name,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodep-kensoku',
    maintainer_email='your email',
    description='Bringup launch package',
    license='Apache License 2.0',
)


