#!/bin/bash

set -eu

## for installing additional non-ROS2 dependencies to debian package generated by bloom
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        echo "[INFO] Initialize rosdep"
        sudo rosdep init
fi

mod_dir=$(echo ${1} | sed 's/\/$//')
yamlpath=${mod_dir}/packaging/rosdep.yaml
# Find ROS_DISTRO text in the yaml file and replace it with env variable
if [ -e ${yamlpath} ]; then
        echo "[INFO] Replace ROS_DISTRO in rosdep.yaml"
        sudo sed -i "s/ROS_DISTRO/${ROS_DISTRO}/g" ${yamlpath}
        echo "[INFO] Add module specific dependencies"
        cat $yamlpath
        mkdir -p /etc/ros/rosdep/sources.list.d
        echo "yaml file://${yamlpath}" > /etc/ros/rosdep/sources.list.d/51-fogsw-module.list
fi

echo "[INFO] Updating rosdep"
rosdep update

apt update
echo "[INFO] Running rosdep install.."
ls ${mod_dir}
if rosdep install --from-paths ${mod_dir} -r -y --rosdistro ${ROS_DISTRO} 1> /dev/null 2>&1; then
	echo "[INFO] rosdep install finished successfully."
else
	echo "[ERROR] Some dependencies missing."
fi

exit 0
