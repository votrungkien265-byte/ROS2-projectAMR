from setuptools import setup

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='trungkien',
    author_email='your_email@example.com',
    description='ROS2 Python node để điều khiển động cơ qua Arduino Mega',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = motor_driver.motor_controller_node:main',
        ],
    },
)