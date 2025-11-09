from setuptools import find_packages, setup

package_name = 'sps30_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['sps30_publisher', 'sps30_publisher.*', 'sps30', 'sps30.*']),  # Ensure submodules are included
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='boris.nikolovski99@gmail.com',
    description='ROS2 package for SPS30 sensor integration (PUBLISHER)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sps30_publisher = sps30.sps30_publisher:main'
        ],
    },
)
