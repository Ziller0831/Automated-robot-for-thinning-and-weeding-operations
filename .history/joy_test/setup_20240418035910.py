from setuptools import find_packages, setup

package_name = 'joy_test'

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
    description='Python Joycontroller Test',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'JoyController = joy_test.JoyController:main'
            ''
        ],
    },
)
