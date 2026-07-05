from setuptools import find_packages, setup

package_name = 'r2_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafie',
    maintainer_email='rafie.alifs@gmail.com',
    description='PBVS pick alignment controller exposed as the AlignAndPick action.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'pick_servo_node = r2_servo.pick_servo_node:main',
        ],
    },
)
