from glob import glob

from setuptools import find_packages, setup


setup(
    name='g1_motion_demo',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/g1_motion_demo']),
        ('share/g1_motion_demo', ['package.xml', 'README.md']),
        ('share/g1_motion_demo/launch', glob('launch/*.launch.py')),
        ('share/g1_motion_demo/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Amadeus',
    maintainer_email='amadeus@example.com',
    description='G1 motion playground for RViz',
    license='Apache-2.0',
    entry_points={'console_scripts': ['motion_demo = g1_motion_demo.node:main']},
)
