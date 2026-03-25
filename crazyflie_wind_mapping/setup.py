from setuptools import find_packages, setup

package_name = 'crazyflie_wind_mapping'

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
    maintainer='william-gao',
    maintainer_email='wgao2005007@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mock_wind_sensor = crazyflie_wind_mapping.mock_wind_sensor:main',
            'real_wind_sensor = crazyflie_wind_mapping.real_wind_sensor:main',
            'wind_field_mapper = crazyflie_wind_mapping.wind_field_mapper:main',
        ],
    },
)
