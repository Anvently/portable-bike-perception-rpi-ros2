from setuptools import setup, find_packages
import os
from glob import glob
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
		('share/' + package_name + '/launch', ['launch/cyclosafe.launch.py', 'launch/config.py']),
		
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manu',
    maintainer_email='pirard.nicolas@hotmail.fr',
    description='Contient les noeuds capteurs en python du projet cyclosafe + la launch description principal pour lancer les mesures',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar = cyclosafe.sonar:main',
			'sonar_rs232 = cyclosafe.sonar_rs232:main',
			'camera_pi = cyclosafe.camera_pi:main',
			'camera_webcam = cyclosafe.camera_webcam:main',
			'gps = cyclosafe.gps:main',
			'sonar_sr04 = cyclosafe.sonar_sr04:main',
			'sonar_lv_pw = cyclosafe.sonar_lv_pw:main',
			'tof_lidar = cyclosafe.tof_lidar:main',
        ],
    },
)
