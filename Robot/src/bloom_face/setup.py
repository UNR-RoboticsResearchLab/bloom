from setuptools import setup
import os
from glob import glob

package_name = 'bloom_face'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/visual_aids', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max Knaefler',
    maintainer_email='mknaefler@unr.edu',
    description='Bloom face display node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'face_node = bloom_face.face_node:main',
        ],
    },
)
