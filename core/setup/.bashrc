parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

source $parent_path/.env

echo "Cyclosafe config:"
echo "workspace: $CYCLOSAFE_WORKSPACE"
echo "record directory: $CYCLOSAFE_RECORD"
echo "log directory: $CYCLOSAFE_LOGS"
echo "scripts path: $SCRIPTS_PATH"
echo "gps serial port: $GPS_SERIAL_PORT"
echo "shutdown delay: $SHUTDOWN_DELAY"
echo "led brightness: $LED_BRIGHTNESS"
echo "low battery percent: $LOW_BATTERY_PERCENT"
echo "low storage treshold: $LOW_STORAGE_TRESHOLD MB"

export CYCLOSAFE_RECORD=$CYCLOSAFE_RECORD

if [ -f $CYCLOSAFE_WORKSPACE/install/setup.bash ]; then
    source $CYCLOSAFE_WORKSPACE/install/setup.bash
	ROS_INSTALLED=1
	CYCLOSAFE_READY=1
  elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
	echo "Ros was sourced but cyclosafe does not appear to be builded"
	echo "Use cy_core_build to build workspace."
	ROS_INSTALLED=1
	CYCLOSAFE_READY=0
  else
	echo "ROS2 Jazzy is not installed"
	ROS_INSTALLED=0
	CYCLOSAFE_READY=0
fi

MSG="Ros2 Jazzy is not installed or the cyclosafe environment was not sourced. If using a different ROS distro, some features may not work properly."

if [ $ROS_INSTALLED -eq 1 ]; then
	alias cy_core_build="cd $CYCLOSAFE_WORKSPACE; colcon build --symlink-install --parallel-workers=2; source ./setup/.bashrc; cd -"
	alias cy_core_clean="cd $CYCLOSAFE_WORKSPACE; rm -rf build/ install/ log/; source ./setup/.bashrc; cd -"
else
	alias cy_core_build="echo $MSG"
	alias cy_core_clean="echo $MSG"
fi
