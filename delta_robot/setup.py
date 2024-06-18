from setuptools import find_packages, setup

package_name = 'delta_robot'

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
    maintainer='ced',
    maintainer_email='wesley222666@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delta_robot_api = delta_robot.DeltaRobot_API:main',
            'trajectory_plan = delta_robot.Trajectory_plan:main'
        ],
    },
)
