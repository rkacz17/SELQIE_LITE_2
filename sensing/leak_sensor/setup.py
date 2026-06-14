from setuptools import find_packages, setup

package_name = 'leak_sensor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SELQIE',
    maintainer_email='rykaczmarczyk@gmail.com',
    description='Leak Sensor Driver Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leak_sensor_node = leak_sensor.leak_sensor_node:main',
        ],
    },
)
