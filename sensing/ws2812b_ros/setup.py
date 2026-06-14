from setuptools import setup, find_packages

package_name = 'ws2812b_ros'

setup(
    name=package_name,
    version='0.0.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'spidev',
        'rclpy',
        'std_msgs',
    ],
    zip_safe=True,
    maintainer='Ryan Kaczmarczyk',
    maintainer_email='rykaczmarczyk@gmail.com',
    description='WS2812B (NeoPixel) SPI driver node for ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'led_node = ws2812b_ros.led_node:main',
        ],
    },
)
