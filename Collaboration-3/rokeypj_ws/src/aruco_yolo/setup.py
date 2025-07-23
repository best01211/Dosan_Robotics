from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'aruco_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/calibration_params.yaml','config/camera_params.yaml','config/kinematics.yaml']), 
        ('share/' + package_name + '/models', ['models/yolov8s_trained.pt']), 

        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey teacher',
    maintainer_email='kimrujin32@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_yolo.aruco_detector:main',
            'aruco_move = aruco_yolo.aruco_move:main',
            'moveit_client = aruco_yolo.moveit_client:main',
            'pick_n_place = aruco_yolo.pick_n_place:main',
            'pick_and_place = aruco_yolo.pick_and_place:main',            
            'compressed_image_pub = aruco_yolo.compressed_image_pub:main',
            'yolo_detector = aruco_yolo.yolo_detector:main',
            'camera_info = aruco_yolo.camera_info:main',
            'camera_pub = aruco_yolo.camera_pub:main',                        
        ],
    },
    package_data={
        package_name: [
            'config/calibration_params.yaml', 
            'models/yolov8s_trained.pt', 
        ],
    },
  
)
