from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cobot_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),  # Bao gồm tất cả các package trừ thư mục test
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'cobot_hardware'), glob('cobot_hardware/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tengsudo',
    maintainer_email='tengsudo.vn@gmail.com',
    description='Package to connect with robot hardware',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_connection = cobot_hardware.robot_connection:main',
            'controller_joint_state = cobot_hardware.controller_joint_state:main',
        ],
    },
)
