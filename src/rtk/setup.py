from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rtk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') + glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'numpy',
        'vosk',
        'sounddevice',
        'gTTS',
        'playsound==1.2.2',
    ],
    zip_safe=True,

    maintainer='apollo930',
    maintainer_email='adityau930@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'manual_control = rtk.manual_control:main',
        'steps_terminal_input = rtk.steps_terminal_input:main',
        'serial_step_bridge = rtk.serial_step_bridge:main',
        'inverse_kinematics = rtk.inverse_kinematics:main',
        'object_detection = rtk.object_detection:main',
        'stt = rtk.stt:main',
    ],
    },
)
