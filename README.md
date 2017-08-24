# l2bot

ROS interface package for converting the output of `/rb_drive/rb_drive/twist_cmd` to a motor controller command and sending it to an L2Bot.

# Installation

If you haven't already, create a catkin workspace.

```
$ mkdir -p ~/l2bot_ws/src
$ cd ~/l2bot_ws/src
```


## Arduino

*Detailed instructions [here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).*

Download and install the [Arduino IDE](https://www.arduino.cc/en/Main/Software).

Install ROS Support. Make sure to replace `indigo` with the correct ROS version name.

```
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial

```

Clone the serial drivers

```
cd ~/l2bot_ws/src
git clone https://github.com/ros-drivers/rosserial.git
cd ~/l2bot_ws
catkin_make
```

In the steps below, `<sketchbook>` is the directory where the Linux Arduino environment saves your sketches. Typically this is a directory called sketchbook in your home directory.


```
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

Restart the Arduino IDE. After restarting, you should see `ros_lib` listed under examples.

![ros_lib examples](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup?action=AttachFile&do=get&target=arduino_ide_examples_screenshot.png)

Load the 'ardunio/L2Bot_MC' sketch onto the L2Bot arduino.


## ROS


Clone this repository into the `src` directory.

```
~/l2bot_ws/src$ git clone https://github.com/LTU-AutoEV/l2bot.git
```

Run `catkin_make` from the workspace directory.

```
~/l2bot_ws/src$ cd ..
~/l2bot_ws$ catkin_make
```

Install dependencies

```
~/l2bot_ws$ rosdep install l2bot
```


# Topics

## Published topics for internal use

  - `std_msgs::UInt16` on **/L2Bot/mc**: Motor controller instruction for the l2bot
