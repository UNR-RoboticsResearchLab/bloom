from setuptools import find_packages, setup

package_name = 'bloom_speech'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),  
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/speech.launch.py']),
        ('share/' + package_name, ['.env']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bloom',
    maintainer_email='mknaefler@unr.edu',
    description='TTS and LLM ROS2 nodes for the Bloom robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tts_node = bloom_speech.tts_node:main',
            'llm_node = bloom_speech.llm_node:main',
            'stt_node = bloom_speech.stt_node:main',
        ],
    },
)