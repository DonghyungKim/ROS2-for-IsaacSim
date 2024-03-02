from setuptools import find_packages, setup

package_name = 'isaacsim_traj_generator'

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
    maintainer='Donghyung Kim',
    maintainer_email='donghyungkim@etri.re.kr',
    description='Trajectory generator for IsaacSim',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaacsim_traj_generator_node = isaacsim_traj_generator.isaacsim_traj_generator:main'
        ],
    },
)
