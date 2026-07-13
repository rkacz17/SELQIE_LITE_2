from setuptools import find_packages, setup

package_name = 'cubemars_calibration'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/calibration.launch.py']),
        ('share/' + package_name + '/config', ['config/calibration.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SELQIE',
    maintainer_email='rykaczmarczyk@gmail.com',
    description='Automatic PD gain calibration for CubeMars AK-series motors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = cubemars_calibration.calibration_node:main',
        ],
    },
)
