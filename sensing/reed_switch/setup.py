from setuptools import find_packages, setup

package_name = 'reed_switch'

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
    description='Reed Switch Driver Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reed_switch_node = reed_switch.reed_switch_node:main',
        ],
    },
)
