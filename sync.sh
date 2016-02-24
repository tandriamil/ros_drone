rsync -e "ssh -i $PIKEYPATH" \
--verbose --links --delete --compress --recursive --times --human-readable --no-perms \
--exclude="CMakeFiles" \
--exclude="CMakeLists.txt" \
catkin_workspace/devel/setup.* \
catkin_workspace/src/pikopter \
pi@10.5.5.1:/home/pi/ros_drone