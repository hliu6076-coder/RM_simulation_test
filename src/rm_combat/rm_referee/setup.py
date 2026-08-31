from setuptools import find_packages, setup

package_name = 'rm_referee'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/combat_rules.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/combat_system.launch.py',
            'launch/lan_referee_demo.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='wangxiaotao',
    maintainer_email='wangxiaotao@example.com',
    description='Authoritative referee for logical RoboMaster combat.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'referee_node = rm_referee.referee_node:main',
            'shoot_cli = rm_referee.shoot_cli:main',
            'combat_teleop = rm_referee.combat_teleop:main',
            'combat_cmd_vel_mux = rm_referee.cmd_vel_mux:main',
            'referee_lan_gateway = rm_referee.lan_gateway:main',
            'referee_lan_client = rm_referee.lan_client:main',
            'navigation_adapter = rm_referee.navigation_adapter:main',
            'combat_monitor = rm_referee.combat_monitor:main',
        ],
    },
)
