from setuptools import setup

package_name = 'bloom_face_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bloom',
    maintainer_email='mknaefler@unr.edu',
    description='Local WebSocket/HTTP bridge re-broadcasting the bloom_face ROS topics to the vizij browser face',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = bloom_face_bridge.bridge_node:main',
        ],
    },
)
