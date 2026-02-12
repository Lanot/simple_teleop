from setuptools import find_packages, setup

package_name = 'simple_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='Anatolii Lehkyi',
    maintainer_email='lanot.biz@gmail.com',
    description='ROS2 simple_teleop package for camera calibration purposes ',
    license='No License',
    install_requires=[
        "pynput==1.8.1"
    ],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop = simple_teleop.teleop:main'
        ],
    },
)
