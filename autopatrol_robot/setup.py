from setuptools import find_packages, setup
from glob import glob

package_name = 'autopatrol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", glob('launch/*.launch.py')),
        ('share/' + package_name + "/config", ['config/patrol_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rayray',
    maintainer_email='rayray@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node = autopatrol_robot.patrol_node:main',
            'speaker = autopatrol_robot.speaker:main',
        ],
    },
)
