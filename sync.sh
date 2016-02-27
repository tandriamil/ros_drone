rsync -e "ssh -i $PIKEYPATH" \
--verbose --links --delete --compress --recursive --times --human-readable --no-perms \
--exclude=".git" \
--exclude=".gitignore" \
--exclude=".catkin_workspace" \
--exclude="sync.sh" \
--exclude="build" \
--exclude="devel" \
--exclude="CMakeFiles" \
--exclude="src/CMakeFiles" \
--exclude="src/CMakeCache.txt" \
--exclude="src/CMakeLists.txt" \
--exclude="src/devel" \
--exclude="src/pikopter/CMakeFiles" \
catkin_workspace \
pi@10.5.5.1:/home/pi/ros_drone
