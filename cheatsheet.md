
# Docker
## Start
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
roslaunch ouster_ros sensor.launch sensor_hostname:=169.254.XX.XXX viz:=true
## My pkg
roslaunch mitacs mitacs_ouster.launch sensor_hostname:=169.254.X.X

# Copy
docker cp ros_ouster:./real_lidar_50hz.csv ~
