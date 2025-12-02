from setuptools import setup
import os
from glob import glob

package_name = 'web_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={
        package_name: [
            'static/*',
            'static/js/*',
            'static/css/*',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), 
            glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'tools'), 
            glob('tools/*.bat') + glob('tools/*.ps1')),
        (os.path.join('share', package_name, 'docs'), 
            glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinshuxin',
    maintainer_email='jinshuxin@example.com',
    description='Web-based monitoring system for TK Chest Controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_monitor_node = web_monitor.web_monitor_node:main',
        ],
    },
)