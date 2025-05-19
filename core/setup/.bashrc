source $CYCLOSAFE_WORKSPACE/setup/.env

if [ -f $CYCLOSAFE_WORKSPACE/install/setup.bash ]; then
    source $CYCLOSAFE_WORKSPACE/install/setup.bash
	ROS_INSTALLED=1
	CYCLOSAFE_READY=1
  elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
	echo "Ros was sourced but cyclosafe does not appear to be builded"
	ROS_INSTALLED=1
	CYCLOSAFE_READY=0
  else
	echo "ROS2 Jazzy is not installed"
	ROS_INSTALLED=0
	CYCLOSAFE_READY=0
fi

local MSG="Ros is not installed or the cyclosafe environment was not sourced."

if [ $ROS_INSTALLED -e 1 ]; then
	alias cy_core_build="cd $CYCLOSAFE_WORKSPACE; colcon build --symlink-install --parallel-workers=2; source $CYCLOSAFE_WORKSPACE/install/setup.bash; cd -"
else
	alias cy_core_build="$MSG"
fi
