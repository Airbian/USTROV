from glob import glob
from setuptools import find_packages, setup

setup(
    name='ustrov_status_display',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ustrov_status_display']),
        ('share/ustrov_status_display', ['package.xml', 'README.md']),
        ('share/ustrov_status_display/launch', glob('launch/*.launch.py')),
        ('share/ustrov_status_display/config', glob('config/*.yaml')),
        ('share/ustrov_status_display/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='USTROV',
    maintainer_email='jwangma@ustrov.local',
    description='Read-only onboard SSD1306 status display.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'status_display_node = ustrov_status_display.node:main',
    ]},
)
