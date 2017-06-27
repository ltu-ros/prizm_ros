# l2bot

ROS interface package for converting the output of
`/rb_drive/rb_drive/twist_cmd` to a motor controller command and sending it to
an L2Bot.

# Installation

## Arduino

Download and install the [Arduino IDE](https://www.arduino.cc/en/Main/Software). Load the 'ardunio/L2Bot_MC' sketch onto the L2Bot arduino.

## ROS

If you haven't already, create a catkin workspace.

```
$ mkdir -p ~/ltumcs_autoev/src
$ cd ~/ltumcs_autoev/src
```

Clone this repository into the `src` directory.

```
~/ltumcs_autoev/src$ git clone https://github.com/LTU-AutoEV/l2bot.git
```

Run `catkin_make` from the workspace directory.

```
~/ltumcs_autoev/src$ cd ..
~/ltumcs_autoev$ catkin_make
```

Install dependencies

```
~/ltumcs_autoev$ rosdep install l2bot
```


# Topics

## Published topics for internal use

  - `std_msgs::UInt16` on **/L2Bot/mc**: Motor controller instruction for the l2bot
