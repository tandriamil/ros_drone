rsync -e "ssh -i ../config_rpi/id_rsa27" \
--verbose --links --delete --compress --recursive --times --human-readable --no-perms \
--exclude=".git" \
--exclude=".gitignore" \
--exclude="sync.sh" \
--exclude="catkin_workspace/build" \
--exclude="devel" \
--include="devel/setup.bash" \
--include="devel/setup.sh" \
--include="devel/setup.zsh" \
catkin_workspace \
pi@raspberrypi.local:/home/pi/ros_drone
