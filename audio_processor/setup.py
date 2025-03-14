from setuptools import find_packages, setup

package_name = 'audio_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuki',
    maintainer_email='yuki.nakagawa@intel.com',
    description='Audio processing node for STT using OpenVINO',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_processor_node = audio_processor.audio_processor_node:main',
        ],
    },
)

