from setuptools import find_packages, setup

package_name = 'mission_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['mission_fsm/config/areas.yaml']),
    ],
    package_data={
        package_name: ['config/areas.yaml'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafie',
    maintainer_email='rafie.alifs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fsm_node = mission_fsm.fsm_node:main',
            'test_signal_pub = mission_fsm.test_signal_pub:main',
            'mock_nav_server = mission_fsm.mock_nav_server:main',
            'forest_executor = mission_fsm.forest_executor_node:main',
            'teensy_command = mission_fsm.teensy_command_node:main',
        ],
    },
)
