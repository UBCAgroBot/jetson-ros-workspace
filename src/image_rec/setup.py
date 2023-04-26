from setuptools import setup

package_name = 'image_rec'

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
    description='Package for Applied AI (Image Rec) Subteam',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'preprocessor = image_rec.pre_processor:main',
            'realsense = image_rec.realsense:main',
            'testpublisher = image_rec.test_imgsrc:main',
            'frcnn = image_rec.frcnn:main',
            'yolo = image_rec.yolo:main',
            'test_imgpost = image_rec.test_imgpost:main',
            'visualization = image_rec.visualization_node:main',
            'overlay_node = image_rec.overlay_node:main',
        ],
    },
)
