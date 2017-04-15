# dashgo D1

Default Branch: slam_02

*Author: Yuxiang Gao*

*Email: yuxianggao96@gmail.com*

## Prerequisites

Omitted

## GMapping

```
roslaunch dashgo_nav gmapping_demo.launch
rosrun dashgo_bringup teleop_twist_keyboard.py
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

Afterwards, save the map using these commands:

```
roscd dashgo_nav/maps
rosrun map_server map_saver -f my_map
```

## Navigation



1. **Robot Bring-up**:

   For minimal bingup:

   `roslaunch dashgo_bringup minimal.launch`

   For smoother performance:

   `roslaunch dashgo_bringup bringup_smoother.launch`

   For obstacle avoidance with sonar sensors (yet to be done) (meiniaoyong):

   ​

2. **AMCL** (Change the `map` file in the launch file if needed):

   * Fire up the AMCL:

     `roslaunch dashgo_nav teb_amcl_demo.launch`

   * To monitor in `rviz` (if you will proceed to the next step, this can be skipped):

     ``rosrun rviz rviz -d `rospack find dashgo_nav`/rviz/amcl.rviz``

     or

     `roslaunch turtlebot_rviz_launchers view_navigation.launch`

3. **Voice Command**:

     * Start AIUI serial communication:

       `roslaunch aiui_speech voice_nav_commands.launch`

     * Start Command listener:

       `roslaunch aiui_speech dashgo_voice_nav.launch`

4. **Navigation**:

     * Initiate navigation along the designated route:

       `roslaunch dashgo_nav tb_nav_test.launch`

     * To monitor in `rviz` :

       ``rosrun rviz rviz -d `rospack find dashgo_nav`/rviz/nav_test.rviz``

5. **Simulation in ArbotiX** (requires rbx1 from github):

     `roslaunch rbx1_nav fake_nav_test.launch`

     ``rosrun rviz rviz -d `rospack find rbx1_nav`/amcl.rviz``




## Web GUI

`roslaunch dashgo_gui rosbridge.launch`

goto http://localhost:8181/robot_gui.html in the web browser on the local computer or use the ip address to access from any device in LAN. 			


​		
​	
