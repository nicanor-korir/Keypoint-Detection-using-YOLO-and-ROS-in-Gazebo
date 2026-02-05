from setuptools import find_packages, setup

package_name = 'advanced_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', ['data/yolov8n-pose.pt']),
        ('share/' + package_name + '/rviz', ['rviz/yolo_pose_estimation.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_pose_estimation_node = advanced_perception.yolo_pose_estimation:main'
        ],
    },
)
