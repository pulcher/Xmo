import os
from glob import glob
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_name = 'xmo_py'

d = generate_distutils_setup(
    packages=['xmo_shared_py'],
    package_dir={'': 'src'}
)

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('../../config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pulcher@killercomputing.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackerman_drive_node = xmo_py.ackerman_drive_node:main',
            'servo_driver = xmo_py.servo_driver:main',
            'servo_node = xmo_py.servo_node:main',
            'camera_position_node = xmo_py.camera_position_node:main'
        ],
    },
)
