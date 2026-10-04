from glob import glob

from setuptools import find_packages, setup

package_name = 'thesis_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
        (
            'share/' + package_name + '/udev',
            glob('udev/*.rules'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Munoz',
    maintainer_email='a20216480@pucp.edu.pe',
    description=(
        'Physical JACO2 orchestration and readiness checks for the thesis.'
    ),
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hardware_readiness = '
            'thesis_hardware.hardware_readiness_node:main',
        ],
    },
)
