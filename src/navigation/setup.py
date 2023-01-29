import os
from glob import glob
from setuptools import setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UBC Agrobot',
    maintainer_email='ubcagrobot@gmail.com',
    description='Package for navigation of Agrobot robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_camera_publisher = navigation.mock_camera_publisher:main',
            'algorithm_publisher = navigation.algorithm_publisher:main',
            'post_processor_publisher = navigation.post_processor_publisher:main'
        ]
    },
)
