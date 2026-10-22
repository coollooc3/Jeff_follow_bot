from glob import glob
from setuptools import setup

setup(
    name='leap_patrol', version='0.1.0', packages=['leap_patrol'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/leap_patrol']),
        ('share/leap_patrol', ['package.xml', 'README.md', 'VALIDATION.md', 'LICENSE']),
        ('share/leap_patrol/config', glob('config/*.yaml')),
        ('share/leap_patrol/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='Jeff', maintainer_email='328209939+coollooc3@users.noreply.github.com',
    description='Nav2 patrol application and scheduling exercise', license='MIT',
    entry_points={'console_scripts': [
        'patrol_manager = leap_patrol.node:main',
        'patrol_lesson = leap_patrol.lesson:main',
        'patrol_check = leap_patrol.exercise_check:main',
    ]},
)
