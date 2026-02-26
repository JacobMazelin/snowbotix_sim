from setuptools import setup

package_name = 'snowbotix_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacob',
    maintainer_email='jacobmaz@umich.edu',
    description='Example control node for Snowbotix Sim',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vehicle_controller = snowbotix_control.vehicle_controller:main',
        ],
    },
)
