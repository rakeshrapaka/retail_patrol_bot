from setuptools import setup

package_name = 'smart_retail_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='Rakesh',
    maintainer_email='rakeshrapaka@gmail.com',
    description='Perception for Smart Retail Bot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'person_detector = smart_retail_perception.person_detector:main',
        ],
    },
)
