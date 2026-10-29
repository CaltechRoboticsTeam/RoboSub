from setuptools import setup
from glob import glob

package_name = 'robosub_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['data/*.json', 'data/*.md']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Caltech Robotics Team',
    maintainer_email='crt@caltech.edu',
    description='Minimal RoboSub AUV simulation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'thruster_mixer = robosub_sim.thruster_mixer:main',
            'actuator_commands = robosub_sim.actuator_commands:main',
            'depth_sensor = robosub_sim.depth_sensor:main',
            'dvl_sensor = robosub_sim.dvl_sensor:main',
            'state_estimator = robosub_sim.state_estimator:main',
            'depth_heading_controller = robosub_sim.depth_heading_controller:main',
            'mission = robosub_sim.mission:main',
            'dashboard = robosub_sim.dashboard:main',
        ],
    },
)
