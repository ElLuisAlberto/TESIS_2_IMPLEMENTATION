from glob import glob
from setuptools import find_packages, setup


package_name = 'thesis_telemetry'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luisito',
    maintainer_email='luisito@todo.todo',
    description='CSV telemetry for the predictive safety pipeline.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'telemetry_logger = thesis_telemetry.telemetry_logger:main',
        ],
    },
)
