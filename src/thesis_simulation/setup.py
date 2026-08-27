from glob import glob
from setuptools import find_packages, setup

package_name = 'thesis_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
        ),
        (
            'share/' + package_name + '/urdf',
            glob('urdf/*.xacro')
        ),
        (
            'share/' + package_name + '/worlds',
            glob('worlds/*.sdf')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luisito',
    maintainer_email='luisito@todo.todo',
    description='Simulation adapters and virtual test environments for the thesis platform.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_command = thesis_simulation.test_command_node:main',
        ],
    },
)
