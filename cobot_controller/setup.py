from setuptools import setup
import os
from glob import glob

package_name = 'cobot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'cobot_controller'), glob('cobot_controller/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TengSudo',
    maintainer_email='tengsudo.vn@gmail.com',
    description='Controller interface node for MoveIt',
    license='612M',
    entry_points={
        'console_scripts': [
            'real_arm_controller = cobot_controller.real_arm_controller:main',
            'fake_arm_controller = cobot_controller.fake_arm_controller:main',
        ],
    },
)
