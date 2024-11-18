from setuptools import find_packages, setup

package_name = 'yolo_segmentation_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'rclpy',
        'opencv-python-headless',
        'ultralytics',
        'pycuda',
        'cv_bridge'
    ],
    zip_safe=True,
    maintainer='tan',
    maintainer_email='tanishq.bhansali49@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'yolo_segmentation_node = yolo_segmentation_node.vision:main'
    ],
},
)
