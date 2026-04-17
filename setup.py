from glob import glob
from setuptools import find_packages, setup

package_name = 'cr10_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
        (f'share/{package_name}/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ser_raneems',
    maintainer_email='ser_raneems@users.noreply.github.com',
    description='ROS 2 Python package for CR10 kinematics, Cartesian motion, and parametric pick-and-place visualization in RViz.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cr10_ik_rviz_node = cr10_project.cr10_ik_rviz_node:main',
            'cr10_pick_place = cr10_project.cr10_pick_place:main',
        ],
    },
)
