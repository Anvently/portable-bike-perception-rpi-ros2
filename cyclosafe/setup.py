from setuptools import setup, find_packages

package_name = 'cyclosafe'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		('share/' + package_name, ['launch/multi_sensor.launch.py']),
    ],
    install_requires=['setuptools', 'picamera2'],
    zip_safe=True,
    maintainer='manu',
    maintainer_email='pirard.nicolas@hotmail.fr',
    description='Contient les différents nodes du projet cyclosafe',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor = cyclosafe.sensor:main',
			'camera_pi = cyclosafe.camera_pi:main',
			'camera_webcam = cyclosafe.camera_webcam:main',
			'gpio = cyclosafe.gpio:main',
        ],
    },
)
