from setuptools import setup

package_name = 'odom_fusion'

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
    description='Odometry fusion node for Smart Retail Bot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_fusion_node = odom_fusion.ekf_node:main',
        ],
    },
)
