from setuptools import find_packages, setup

package_name = 'humanoid_vla'

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
    maintainer='samia',
    maintainer_email='samia@example.com',
    description='Package for Vision-Language-Action system for humanoid robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_processor = humanoid_vla.vla_processor:main',
        ],
    },
)