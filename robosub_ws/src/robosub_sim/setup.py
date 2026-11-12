from setuptools import setup
from glob import glob
import os
from pathlib import Path

package_name = 'robosub_sim'

def package_files(directory, dest_dir):
    paths = []
    if os.path.exists(directory):
        for path, directories, filenames in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                rel_path = os.path.relpath(file_path, directory)
                target_dir = os.path.join(dest_dir, os.path.dirname(rel_path))
                paths.append((target_dir, [file_path]))
    
    d = {}
    for dest, files in paths:
        d.setdefault(dest, []).extend(files)
    return list(d.items())

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        *package_files('models', 'share/' + package_name + '/models'),
        *package_files('meshes', 'share/' + package_name + '/meshes'),
        *package_files('media', 'share/' + package_name + '/media'),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='crt',
    maintainer_email='timzhang2018@gmail.com',
    description='Minimal RoboSub AUV simulation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'thruster_mixer = robosub_sim.thruster_mixer:main',
            'depth_sensor = robosub_sim.depth_sensor:main',
            'dvl_sensor = robosub_sim.dvl_sensor:main',
            'state_estimator = robosub_sim.state_estimator:main',
            'depth_heading_controller = robosub_sim.depth_heading_controller:main',
            'mission = robosub_sim.mission:main',
            'dashboard = robosub_sim.dashboard:main',
        ],
    },
)