from setuptools import setup

package_name = 'patrol_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rakesh',
    maintainer_email='rakeshrapaka@gmail.com',
    description='Patrol manager node for Smart Retail Bot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node = patrol_manager.patrol_node:main',
        ],
    },
)
