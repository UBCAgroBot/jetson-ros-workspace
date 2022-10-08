from setuptools import setup

package_name = 'navigation'

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
    maintainer='UBC Agrobot',
    maintainer_email='ubcagrobot@gmail.com',
    description='Package for navigation of Agrobot robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_camera_publisher = navigation.mock_camera_publisher:main',
            'center_row_publisher = navigation.center_row_publisher:main'
            'scanning_publisher = navigation.scanning_publisher:main'
            'mini_contours_publisher = navigation.mini_contours_publisher:main'
            'mini_contours_downwards_publisher = navigation.mini_contours_downwards_publisher:main'
            'check_row_end_publisher = navigation.check_row_end_publisher:main'
            'hough_publisher = navigation.hough_publisher:main'
            'post_processing_publisher = navigation.post_processing_publisher:main'
        ]
    },
)
