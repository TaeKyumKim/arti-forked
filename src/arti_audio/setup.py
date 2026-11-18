from glob import glob

from setuptools import find_packages, setup

package_name = 'arti_audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROKEY-10-Arti',
    maintainer_email='arti@todo.invalid',
    description='Arti guide audio: artwork explanation action server and guide voice messages',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'guide_audio_node = arti_audio.guide_audio_node:main',
        ],
    },
)
