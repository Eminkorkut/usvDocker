from setuptools import find_packages, setup

package_name = 'myRos2'

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
    maintainer='emin',
    maintainer_email='emin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "boat_control_with_keyboard = myRos2.boat_control_with_keyboard:main",
            "transfer_camera_frame = myRos2.transfer_camera_frame:main",
            "autonomous_boat_movement = myRos2.autonomous_boat_movement:main",
        ],
    },
)
