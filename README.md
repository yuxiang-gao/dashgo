EAI dashgo D1
slam_02

roslaunch dashgo_bringup minimal.launch
roslaunch dashgo_nav teb_amcl_demo.launch

roslaunch dashgo_nav tb_nav_test.launch
rosrun rviz rviz -d `rospack find rbx1_nav`/nav_test.rviz

roslaunch rbx1_nav fake_nav_test.launch
rosrun rviz rviz -d `rospack find rbx1_nav`/amcl.rviz


roslaunch aiui_speech voice_nav_commands.launch
roslaunch aiui_speech dashgo_voice_nav.launch
