# l2bot

ROS interface package for converting the output of `/rb_drive/rb_drive/twist_cmd` to a motor controller command and sending it to an L2Bot.

# Installation

If you haven't already, create a catkin workspace.

```
$ mkdir -p ~/l2bot_ws/src
$ cd ~/l2bot_ws/src
```


## Arduino

Download and install the [Arduino IDE](https://www.arduino.cc/en/Main/Software).

Install ROS Support: [Arduino Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

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

You may need to change the ID of the arduino in the launch file. To do this, open ~/l2bot_ws/src/l2bot/launch/l2bot.launch and change the line below.

```
<!-- Change the 'value' parameter of the line below -->
<!-- run `ls /dev/serial/bi-id` to get the id of your Arduino -->
<param name="port" value="/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_64934333235351404262-if00"/>
```


# Topics

## Published topics for internal use

  - `std_msgs::UInt16` on **/L2Bot/mc**: Motor controller instruction for the l2bot
