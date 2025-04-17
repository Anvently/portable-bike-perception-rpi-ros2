from setuptools import setup, find_packages
import os

package_name = 'cyclosafe'

# # Rendre le script gpio.sh exécutable
# script_path = 'scripts/gpio.sh'
# if os.path.exists(script_path):
#     os.chmod(script_path, 0o755)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		('share/' + package_name, ['launch/cyclosafe.launch.py', 'launch/test_config.launch.py', 'launch/viewer.launch.py']),
        ('share/' + package_name, ['launch/view.rviz', ]),
		('share/' + package_name, ['scripts/gpio.py', 'scripts/gpio.sh']),
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
            'sonar = cyclosafe.sonar:main',
			'sonar_rs232 = cyclosafe.sonar_rs232:main',
			'camera_pi = cyclosafe.camera_pi:main',
			'camera_webcam = cyclosafe.camera_webcam:main',
			'gps = cyclosafe.gps:main',
			'image_viewer = cyclosafe.image_viewer:main',
			'range_circle_transform = cyclosafe.range_circle_transform:main',
			'sonar_sr04 = cyclosafe.sonar_sr04:main',
        ],
    },
)
