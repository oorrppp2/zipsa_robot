# Execute the Demo

The demo consists of 6 steps: Introduction of myself, Introduction of Living Room, Introduction of Smart Kitchen, Finalize the Demo, Ready for Grasp, Grasp the Gift and Move to Door.

After the bringup procedure, You can control the demo by sending topic to robot.


## Command to play the step of Demo.

### 1. Introduction of myself

**Description**: ZipSa turns own body rotate joint to looks backside, and wait trigger signal. After receive the start trigger signal, ZipSa will say the introduction of myself.

    $ rostopic pub -1 /wait_select_scene std_msgs/String "data: 'intro'"
    $ rostopic pub -1 /wait_start_trigger std_msgs/String "data: 'start'"


### 2. Introduction of Living Room

**Description**: ZipSa will move to the some point in the living room. And, it will said the introduction of living room.

    $ rostopic pub -1 /wait_select_scene std_msgs/String "data: 'livingroom'"


### 3. Introduction of Smart Kitchen

**Description**: ZipSa will move to the some point in the kitchen, and look smart table by rotating head (pan/tilt). And, it will say the introduction of smart kitchen.

    $ rostopic pub -1 /wait_select_scene std_msgs/String "data: 'kitchen'"


### 4. Finalize the Demo

**Description**: ZipSa will move to the home position, and will say thanks for watching demo.

    $ rostopic pub -1 /wait_select_scene std_msgs/String "data: 'home_end'"


### 5. Ready for Grasp

**Description**: ZipSa will move to the grasp position.

    $ rostopic pub -1 /wait_select_scene std_msgs/String "data: 'grasp'"


### 6. Grasp the Gift and Move to Door

**Description**: ZipSa will move the arm_base joint to extract the own manipulator, and will look at low position for searching QR code on the wall. After searching the code, if it finds the code, ZipSa will perform grasp procedures for grasp the eco bag which is hang on the wall. And ZipSa will move to position where beside of Door.

    $ rostopic pub -1 /wait_select_scene std_msgs/String "data: 'grasp6'"