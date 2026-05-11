import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rm_75_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eskin',
    maintainer_email='qk.xu@joysonquin.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rm_cali_v1 = rm_75_calibration.rm_cali_v1:main',
            'force_sensor_node = rm_75_calibration.force_sensor_node:main',
            'force_control_z_node = rm_75_calibration.force_control_z_node:main',
            'force_control_xz_node = rm_75_calibration.force_control_xz_node:main',
            'force_control_node = rm_75_calibration.force_control_node:main',
        ],
    },
)
