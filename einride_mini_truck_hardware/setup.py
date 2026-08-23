from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'einride_mini_truck_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # PEP 561: tells a type checker the package ships real annotations, so
    # consumers get them instead of treating every import as Any.
    package_data={package_name: ['py.typed']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vladyslav Aleksashyn',
    maintainer_email='vladyslav.aleksashyn@einride.tech',
    description=(
        'Hardware abstraction layer: drives the real UGV02 chassis over serial '
        "and reproduces the simulation's topic contract"
    ),
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ugv02_serial = einride_mini_truck_hardware.ugv02_serial_node:main',
        ],
    },
)
