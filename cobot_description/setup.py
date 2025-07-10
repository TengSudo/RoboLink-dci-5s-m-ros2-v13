from setuptools import setup
import os
from glob import glob

package_name = 'cobot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cobot_description']),
        ('share/cobot_description', ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'cobot_description'), glob('cobot_description/*')),
    ],

    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TengSudo',
    maintainer_email='tengsudo.vn@gmail.com',
    description='ROS2 Humble package for cobot description',
    license='612M',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
