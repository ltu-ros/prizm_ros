# `prizm_ros`

Unofficial ROS interface package for Tetrix PRIZM motor controller. See below for setup/installation


## Interfaces

### `prizm`

![Prizm Topics](doc/prizm.png)

```
$ roslaunch prizm prizm.launch
```

This launch file provides the basic motor controller functionality  

Topics:

  - Publish a `geometry_msgs::Point` on `~/motor_cmd` as the motor controller instruction
    - `x`: motor 1, `y`: motor 2
    - `x` and `y` values must range from `-100` to `100`
  - Publish a `std_msgs::Bool` on `~/red_led` to set the state of the red LED
  - Publish a `std_msgs::Bool` on `~/green_led` to set the state of the green LED

### `twist_controller`

![Twist controller topics](doc/prizm_twist.png)

This launch file converts twist messages into direct motor commands.

Topics:

  - Publish a `geometry_msgs::Twist` on `~/twist_controller/twist_cmd` to drive the robot
    - `twist.linear.x`: forward/backward velocity (-3.0 to 3.0)
    - `twist.angular.z`: right/left velocity (-1.5 to 1.5) (approx radians)


# Setup and Installation

Contents (details below):

  1. Setup a ROS  workspace
  2. Install Arduino IDE
  3. Install ROS libs
  4. Install PRIZM libs
  5. Clone this repo into your workspace
  6. Upload the motor controller sketch to the Arduino
  7. Point the ROS node to your board


## 1. Setup Workspace

If you already have a worksapce, skip this step.

Create a catkin workspace.

```
$ mkdir -p ~/prizm_ws/src
$ cd ~/prizm_ws/src
```


## 2. Arduino

Follow instructions [here](https://www.arduino.cc/en/guide/linux) to download ans install Arduino.

## 3. Installing ROS Support

[Arduino Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

Make sure to replace `indigo` with `kinetic` when executing commands. For example, if the pages says to run the command

```
sudo apt-get install ros-indigo-rosserial-arduino
```

then you should run the command

```
sudo apt-get install ros-kinetic-rosserial-arduino
```

## 4. Install PRIZM libs

Download library from here: https://www.pitsco.com/TETRIX-PRIZM-Robotics-Controller

Unzip and place library in ~/Arduino/libraries

## 5. PRIZM package


Clone this repository into the `src` directory.

```
~/prizm_ws/src$ git clone https://github.com/LTU-AutoEV/prizm_ros.git
```

Run `catkin_make` from the workspace directory.

```
~/prizm_ws/src$ cd ..
~/prizm_ws$ catkin_make
~/prizm_ws$ source devel/setup.bash
```

Install dependencies

```
~/prizm_ws$ rosdep install --from-paths src --ignore-src -r -y
```

## 6. Load The Interface onto the Arduino

  1. Open the Arduino IDE
  2. File > Open
    - Open the file `PRIZM_MC.ino` located at `~/prizm_ros/arduino/prizm_MC/` 
  3. Plug in the Arduino and turn on the prizm
  4. Tools -> Port -> (Select your arduino device)
  5. Upload to arduino


## 7. Identify Your Arduino Board

  1. Plug in your Arduino and turn on the prizm
  2. Execute the `detect_arduino.py` script

```
prizm_ws/src/prizm$ ./detect_arduino.py
```

## 8. Test PRIZM

```
catkin_make
source devel/setup.bash
roslaunch prizm_ros prizm.launch
```

Press the green button on the PRIZM after every `roslaunch`

Run `rostopic list`. You should see `/green_led` and `/red_led` topics.





