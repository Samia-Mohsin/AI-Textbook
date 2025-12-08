from setuptools import find_packages, setup

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/autonomous_humanoid.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samia',
    maintainer_email='samia@example.com',
    description='Package for humanoid robot control system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_humanoid = humanoid_control.autonomous_node:main',
            'voice_interface = humanoid_control.voice_interface:main',
        ],
    },
)