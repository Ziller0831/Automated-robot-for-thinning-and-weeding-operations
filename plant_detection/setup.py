from setuptools import find_packages, setup

package_name = 'plant_detection'

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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plant_detector = plant_detection.plant_detection:main',
            'photo_pub = plant_detection.photo_pub:main'
        ],
    },
)
