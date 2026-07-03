from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'line_sensor_data_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'data'), ['data/.gitkeep'] + glob('data/*.jsonl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olagh48652',
    maintainer_email='oghattas@hello-robot.com',
    description='Record and replay Stretch line sensor data for filter tests.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_sensor_record = line_sensor_data_tools.record:main',
            'line_sensor_replay_filter = line_sensor_data_tools.replay_filter:main',
        ],
    },
)
