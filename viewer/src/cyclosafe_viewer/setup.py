from setuptools import find_packages, setup
from glob import glob

package_name = 'cyclosafe_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		('share/' + package_name + '/launch', ['launch/viewer.launch.py', 'launch/config.py']),
		('share/' + package_name + '/launch', ['launch/view.rviz', ]),
        ('share/' + package_name + '/urdf', ['urdf/model.urdf.xml']),
        ('share/' + package_name + '/urdf/model', glob('urdf/model/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='npirard',
    maintainer_email='pirard.nicolas@hotmail.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'gps_converter = cyclosafe_viewer.gps_converter:main',
			'range_circle_transform = cyclosafe_viewer.range_circle_transform:main',
        ],
    },
)
