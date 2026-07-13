# Docker
docker run -it --net=host --ipc=host --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /dev:/dev --privileged -v /home/josch/Projects/Mitacs:/root/catkin_ws/src/mitacs --name ros_ouster_sync ros_ouster_sync bash
## Start
xhost +local:root
export LIBGL_ALWAYS_SOFTWARE=1
docker start -i ros_ouster_sync
docker exec -it ros_ouster_sync bash
## Stop
docker stop ros_ouster_sync

# Ping (serial number is : 122220003768)
ping os-122220003768.local

# Launch lidar
xhost +local:root
export LIBGL_ALWAYS_SOFTWARE=1
## Oster default command
roslaunch ouster_ros sensor.launch sensor_hostname:=169.254.185.245 viz:=true
## My pkg
roslaunch mitacs mitacs_ouster.launch sensor_hostname:=169.254.185.245
# Copy
docker cp ros_ouster:./real_lidar_50hz.csv ~

# Find
find . -iname "*name*"

# Arduino
## To generate the msg libraries
rosrun rosserial_arduino make_libraries.py /tmp
## Make arduino + ros work
roscore
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

# Combine
roslaunch mitacs orbita_ouster_mitacs.launch
rostopic pub /target_orientation geometry_msgs/Vector3 "{x: 20.0, y: 0.0, z: 0.0}"
