from setuptools import setup

package_name = 'cyclosafe'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		('share/' + package_name, ['launch/multi_sensor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manu',
    maintainer_email='pirard.nicolas@hotmail.fr',
    description='Contient les différents nodes du projet cyclosafe',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor = cyclosafe.sensor:main'
        ],
    },
)
