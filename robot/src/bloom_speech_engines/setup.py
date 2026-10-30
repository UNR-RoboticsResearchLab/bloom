from setuptools import find_packages, setup

package_name = 'bloom_speech_engines'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    # Not apt/rosdep packages. Install with: pip install -e 'robot/src/bloom_speech_engines[azure]'
    extras_require={
        'azure': ['azure-cognitiveservices-speech==1.46.0'],
        'llm': ['openai==1.58.1'],
    },
    zip_safe=True,
    maintainer='jrkre',
    maintainer_email='joshuaknight@unr.edu',
    description='Pluggable TTS and LLM engines for the Bloom robot',
    license='MIT',
)
