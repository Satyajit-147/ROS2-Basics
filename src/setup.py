from setuptools import setup, find_packages

package_name = 'number_square_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='satyajit',
    maintainer_email='satyajitgr9@gmail.com',
    description='A simple ROS 2 package with a publisher and subscriber',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'number_publisher = number_square_package.number_publisher:main',
            'square_subscriber = number_square_package.square_subscriber:main',
        ],
    },
)

