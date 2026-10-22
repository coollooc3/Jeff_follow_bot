from glob import glob
from setuptools import setup

setup(
    name='leap_marker_follow', version='0.1.0',
    packages=['leap_marker_follow'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/leap_marker_follow']),
        ('share/leap_marker_follow', ['package.xml', 'README.md', 'LICENSE']),
        ('share/leap_marker_follow/launch', glob('launch/*.launch.py')),
        ('share/leap_marker_follow/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='LEAP developer', maintainer_email='developer@example.com',
    description='Continuous ArUco marker following for LEAP', license='MIT',
    entry_points={'console_scripts': ['marker_follow = leap_marker_follow.node:main']},
)
