from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'einride_mini_truck_application'


def config_files() -> list[tuple[str, list[str]]]:
    """Install config/<subsystem>/*.yaml, keeping the subfolders."""
    entries = []
    for folder in sorted(glob('config/*/')):
        files = glob(os.path.join(folder, '*.yaml'))
        if files:
            entries.append((os.path.join('share', package_name, folder), files))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # config/saga/saga.secret.yaml is installed too, so the token reaches
        # the node, but it is gitignored and never committed.
    ] + config_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sahib Kazimli',
    maintainer_email='sahib.kazimli130@gmail.com',
    description=(
        'Competition application: Saga AI client and mission on top of Nav2, '
        'opennav_docking, apriltag_ros and robot_localization'
    ),
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'saga = einride_mini_truck_application.saga.node:main',
            'wheel_odometry = einride_mini_truck_application.localization.node:main',
            'dock_pose = einride_mini_truck_application.perception.node:main',
            'tag_survey = einride_mini_truck_application.perception.tag_survey:main',
            'mission = einride_mini_truck_application.mission.node:main',
        ],
    },
)
