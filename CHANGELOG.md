# Changelog

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
