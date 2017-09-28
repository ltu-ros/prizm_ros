# l2bot

ROS interface package for converting the output of `/rb_drive/rb_drive/twist_cmd` to a motor controller command and sending it to an L2Bot.

# Installation

If you haven't already, create a catkin workspace.

```
$ mkdir -p ~/l2bot_ws/src
$ cd ~/l2bot_ws/src
```


## Arduino

  2. Download and install the [Arduino IDE](https://www.arduino.cc/en/Main/Software).
  2. Open the IDE and close it once it has finished launching. (This creates all the scketchbook directories on your system)
  3. Install ROS Support (see below)

### Installing ROS Support

[Arduino Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

Make sure to replace `indigo` with `kinetic` when executing commands. For example, if the pages says to run the command

```
sudo apt-get install ros-indigo-rosserial-arduino
```

then you should run the command

```
sudo apt-get install ros-kinetic-rosserial-arduino
```




## ROS


Clone this repository into the `src` directory.

```
~/l2bot_ws/src$ git clone https://github.com/LTU-AutoEV/l2bot.git
```

Run `catkin_make` from the workspace directory.

```
~/l2bot_ws/src$ cd ..
~/l2bot_ws$ catkin_make
~/l2bot_ws$ source devel/setup.sh
```

Install dependencies

```
~/l2bot_ws$ rosdep install --from-paths src --ignore-src -r -y
```

You may need to change the ID of the arduino in the launch file. To find the ID, **plug in the Arduino** and run the following

```
$ ls /dev/serial/by-id/

```

Open `~/l2bot_ws/src/l2bot/launch/l2bot.launch` and change the line below.


```
<!-- Change the 'value' parameter of the line below -->
<param name="port" value="/dev/serial/by-id/YOUR_ID_HERE"/>
```

For example, yours may be:
```
<!-- Change the 'value' parameter of the line below -->
<param name="port" value="/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_64934333235351404262-if00"/>
```

# Topics

## Published topics for internal use

  - `std_msgs::UInt16` on **/L2Bot/mc**: Motor controller instruction for the l2bot
