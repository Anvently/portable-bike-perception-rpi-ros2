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
		('share/' + package_name, ['launch/cyclosafe.launch.py', 'launch/test_config.launch.py', 'launch/viewer.launch.py']),
        ('share/' + package_name, ['launch/view.rviz']),
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
			'camera_pi = cyclosafe.camera_pi:main',
			'camera_webcam = cyclosafe.camera_webcam:main',
			'gpio = cyclosafe.gpio:main',
			'gps = cyclosafe.gps:main',
			'image_viewer = cyclosafe.image_viewer:main',
			'range_circle_transform = cyclosafe.range_circle_transform:main',
        ],
    },
)
