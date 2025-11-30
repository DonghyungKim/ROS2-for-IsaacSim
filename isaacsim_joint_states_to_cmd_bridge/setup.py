from setuptools import find_packages, setup

package_name = 'isaacsim_joint_states_to_cmd_bridge'

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
    maintainer='etri',
    maintainer_email='etri@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaacsim_joint_states_to_cmd_bridge_node = isaacsim_joint_states_to_cmd_bridge.isaacsim_joint_states_to_cmd_bridge:main'
        ],
    },
)
