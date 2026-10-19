# ROC2
ros2 launch mcar_description bringup_sim.launch.py g:=true d_i:=false r:=true k:=false a:=false s_s:=true n:=true

source install/setup.bash

colcon build --packages-select mcar_teleop_task

ros2 run mcar_teleop_task <filename>
