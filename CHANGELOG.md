# Changelog

## [1.2.0] - 2025-08-19

### Added

- **ENCRYPTION**
  - encryptor node to compress and encrypt ros bag as they are created
  - encryptor node use AES algorithm, the key is encrypted with an RSA public key.
    - the only way to decrypt the AES key is to have to RSA private key
- core/.env : `$ENCRYPTION` environment variable can be defined to 1 to enable encryption
- import_recordings : added verbose option to enable `INFO` log level (default is set to WARNING)

### Modified

- cyclosafe.launch.py:
  - if `$ENCRYPTION=1` :
    - rosbag recorder is launched withtout compression
    - encryptor node is launched
  - else
    - rosbag recorder is launched with compression
    - encryptor node is not launched*
- launch_wrapper.sh :
  - adapted `$EXPECTED_NODE` calculation to handle the encryptor node
- .env: `$SHUTDOWN_DELAY` default value increased to 10s in order for the encryptor node to finish its encryptions
- scripts/csv_importer : adapted scripts to be more efficient, simplified module imports
- core/launch_wrapper.sh : number of node is now checked twice

### Fixed

- core/gpio.sh : last warning check to log when shutdown while cyclosafed.service still active was targetting wrong service name

## [1.1.1] - 2025-08-14

### Added
- cyclosafe_viewer : implementd invert_lidar argument to use another urdf model where both lidar transformation are inverted , in case lidar connections are wrong
- various readme : uml modl + simplified node schematic added

### Modified
- design/ : updated BOM

## [1.1.0] - 2025-08-08
### Added
- CHANGELOG.md file
- CONTRIBUTORS.md file
- viewer/cyclosafe_player: visualize gps trace on map widget + live position + gps data curve and live output
- scripts/import_recording: env check in import_recording script
- viewer/dependancies: python3-rich to ROS env (viewer/cyclosafe_viewer/package.json)
- README: section mentionning context of the project
- README: section expliciting what material is concerned by the new CeCILL-B license
- added 3 csv script exporter files in scripts/
- added automatic csv conversion to import_recording script

### Modified
- License CeCILL updated to CeCILL-B (from GPL to BSD) in order to make the project easier to redistribute
- README: 
  - updated project description 
  - translated main page to english
  - new name for the repo to be more exhaustive
- core/gpio.py: changed shutdown button max latency from 0.5s to 0.25s
- viewer/cyclosafe_player: updated image preview
- scripts/README: added recommendation to install cyclosafe env from viewer/README
- core/README : added reference to ROS2.md

### Fix
- cyclosafe_player:
  - missing python3-pyqt5.qtwebengine dependancy in package.json
  - nan check for angle_min, angle_max and angle_increment in lidar message
- core/scripts/gpio.py : missing import pigpio (removed in a previous commit for no reason), missing import sys (non fatal) + intercept exception related to i2c to ensure the script won't crash

## [1.0.0] - 2025-08-06
### Added
- Initial version of repo associated with a compressed firmware image. 
