import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gesture_mouse'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jruzek',
    maintainer_email='jruzek@todo.todo',
    description='Webcam gesture-controlled virtual mouse with ROS 2 integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_publisher = gesture_mouse.gesture_publisher:main',
            'mouse_controller = gesture_mouse.mouse_controller:main',
            'arm_controller = gesture_mouse.arm_controller:main',
        ],
    },
)
