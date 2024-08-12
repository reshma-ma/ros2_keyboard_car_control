from setuptools import setup

package_name = 'car_control'

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
    maintainer='hiwi02',
    maintainer_email='hiwi02@todo.todo',
    description='keyboard input for a car control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'keyboard_input_node = car_control.keyboard_input_node:main',
		'velocity_publisher_node =car_control.velocity_publisher_node:main',
        ],
    },
)
