parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

source $parent_path/.env

echo "Cyclosafe config:"
echo "workspace: $CYCLOSAFE_WORKSPACE"

export CYCLOSAFE_WORKSPACE=$CYCLOSAFE_WORKSPACE
export HOST_IP=$HOST_IP
export RASPI_IP=$RASPI_IP

if [ -f $CYCLOSAFE_WORKSPACE/install/setup.bash ]; then
    source $CYCLOSAFE_WORKSPACE/install/setup.bash
	ROS_INSTALLED=1
  elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
	echo "Ros was sourced but cyclosafe does not appear to be builded"
	echo "Use cy_viewer_build to build workspace."
	ROS_INSTALLED=1
  else
	echo "ROS2 Jazzy is not installed"
	ROS_INSTALLED=0
fi

if [ $ROS_INSTALLED -eq 1 ]; then
	alias cy_viewer_build="cd $CYCLOSAFE_WORKSPACE; colcon build --symlink-install; source ./setup/.bashrc; cd -"
	alias cy_viewer_clean="cd $CYCLOSAFE_WORKSPACE; rm -rf build/ install/ log/; source ./setup/.bashrc; cd -"
else
	alias cy_viewer_build="echo Ros is not installed or the cyclosafe environment was not sourced."
	alias cy_viewer_clean="echo Ros is not installed or the cyclosafe environment was not sourced."
fi
