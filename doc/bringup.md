# Bringup the ZipSa (Living Lab. Robot Version 2)

## Power on

Press the power button which located back side in mobile part.


## Connect to ZipSa by WiFi

AP name: LivingLabRobotv2_ap24
Password: livingap11

    $ ssh robot@192.168.9.10 or ssh robot@robot-v2-livinglab


Register the developt computer hostanem to /etc/hosts


## 1. Bring up the software

Use the icons on Desktop. Just press double touch in the screen. If you want bringup by remote, open the terminal and execute launch file.

    $ roslaunch living_lab_robot_bringup bringup.launch


## 2. Homing

Some joints (body_rotate_joint, elevation_joint, arm_base_joint, pan_joint) are need the homing procedure before use it. After bringup the software, just press the homing button. Or, open another terminal and execute run file.

    $ rosrun living_lab_robot_dynamixel do_homing.py


## 3. Run robot face

ZipSa use the screen to show the speech activity. After 1, 2 step, just press the button face. Or, open another terminal and execute run file.

    $ roslaunch siri_wave_screen bringup.launch


## 4. Run robot apps & webserver

ZipSa use the py_tree (behavior tree framework on ROS) for execute the demo. just press the button apps. Or, open another terminal and execute run file.

    $ roslaunch living_lab_robot_bringup bringup_app.launch