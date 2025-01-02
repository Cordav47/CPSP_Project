from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'drone_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'drone_project.risk_image_publisher',
        'drone_project.standard_image_publisher',
        'drone_project.image_processor',
        'drone_project.goal_sender',
        'drone_project.offboard_control',
        'drone_project.CV_library'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dc7',
    maintainer_email='dc7@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'risk_image_publisher = drone_project.risk_image_publisher:main',
            'standard_image_publisher = drone_project.standard_image_publisher:main',
            'image_processor =  drone_project.image_processor:main',
            'goal_sender = drone_project.goal_sender:main',
            'offboard_control = drone_project.offboard_control:main'
        ],
    },
)
