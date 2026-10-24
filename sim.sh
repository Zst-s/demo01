

cmds=( "ros2 launch rm_nav_bringup bringup_sim.launch.py \
            world:=RMUL \
            mode:=nav \
            lio:=pointlio \
            localization:=amcl \
            lio_rviz:=False \
            nav_rviz:=True"
        "ros2 run py_tools publish_initial_pose 0.0 0.0 0.0 0.0 0.0 0.0 0.0"
)

for cmd in "${cmds[@]}";
do
    echo Current CMD : "$cmd"
    gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;"
    sleep 0.2
done
