from setuptools import setup

package_name = 'sick_tim_usb_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/usb_lidar.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eagle',
    maintainer_email='savyapatel@gmail.com',
    description='Publish LaserScan from SICK TiM561 via USB for slam',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_lidar_node = sick_tim_usb_ros2.usb_lidar_node:main',
        ],
    },
)
