from setuptools import find_packages, setup

package_name = 'thesis_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Munoz',
    maintainer_email='a20216480@pucp.edu.pe',
    description=(
        'Supervised joint-control interface for simulation and physical '
        'JACO2 use.'
    ),
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_gui = thesis_ui.joint_control_gui:main',
        ],
    },
)
