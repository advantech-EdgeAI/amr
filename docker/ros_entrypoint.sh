#!/bin/bash
set -e

for file in $(find /opt/ros -maxdepth 2 -type f -name "setup.bash"); do
    if [ -f "$file" ]; then
        # Use 'source' or '.' to source the script
        source "$file"
        echo "source $file" >> /root/.bashrc
    fi
done
# source "$(find /opt/ros -maxdepth 2 -type f -name "setup.bash")"
export PYTHONPATH=$release:$PYTHONPATH
echo "export PYTHONPATH=$release:$PYTHONPATH" >> /root/.bashrc
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc
export ROS_DOMAIN_ID=0
echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc

. ~/.bashrc
exec "$@"

/bin/bash