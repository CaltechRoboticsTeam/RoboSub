from setuptools import setup
from glob import glob

package_name = 'robosub_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config',
         glob('config/*.yaml') + glob('config/*.gzlaunch')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
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
