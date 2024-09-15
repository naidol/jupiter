from setuptools import find_packages, setup

package_name = 'wheel_joint_state_publisher'

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
    maintainer='logan',
    maintainer_email='naidoo.logan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wheel_joint_state_publisher = wheel_joint_state_publisher.wheel_joint_state_publisher:main"
        ],
    },
)
