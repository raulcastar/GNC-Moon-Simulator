import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'component_robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Raul Castilla Arquillo',
    author_email='raul.castilla@uni.lu',
    maintainer='Raul Castilla Arquillo',
    maintainer_email='raul.castilla@uni.lu',
    keywords=['ROS'],
    description="The simulated rover's frames as static TF, from a URDF exported from Unreal.",
    license='MIT License',
)
