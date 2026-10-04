from setuptools import find_packages, setup

package_name = 'car_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wejde',
    maintainer_email='wejdens49@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
        entry_points={
        'console_scripts': [
            'safety_node = car_control.safety_node:main',
            'lane_detector = car_control.lane_detector:main',
            'lane_controller = car_control.lane_controller:main',
        ],
    },
)
