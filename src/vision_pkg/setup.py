from setuptools import setup, find_packages

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python>=4.5'
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='2154863296@qq.com',
    description='ROS2 visual line tracking package with OpenCV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linetrack_node = vision_pkg.linetrack:main',
            'arrow_detector_node = vision_pkg.arrow_detector:main',
            'detect_node=vision_pkg.detect:main'
        ],
    },
)