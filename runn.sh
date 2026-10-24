#colcon build 
echo "1" | sudo -S sudo chmod 666 /dev/ttyUSB0
# 1

cmds=(  "ros2 launch rm_nav_bringup bringup_real.launch.py \
            world:=111\
            mode:=mapping \
            lio:=pointlio \
            localization:=icp \
            lio_rviz:=False \
            nav_rviz:=True"
        "ros2 run decision trans_node"
        "ros2 run decision rosout_node2"
        #"ros2 run decision decision"
        #"ros2 run decision mytest"
        #"ros2 run py_tools publish_initial_pose 0.0 0.0 0.0 0.0 0.0 0.0 0.0"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done

sleep 12
echo Current CMD : "ros2 launch rm_bt bt_tree.launch.py"
gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;ros2 launch rm_bt bt_tree.launch.py;exec bash;"
sleep 0.2
