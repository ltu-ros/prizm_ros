# l2bot

ROS interface package for converting the output of `/rb_drive/rb_drive/twist_cmd` to a motor controller command and sending it to an L2Bot.

# Installation

Contents (details below):

  1. Setup a ROS  workspace
  2. Install Arduino IDE and ROS support
  3. Clone this repo into your workspace
  4. Upload the motor controller sketch to the Arduino
  4. Point the ROS node to your board


## 1. Setup Workspace

If you haven't already, create a catkin workspace.

```
$ mkdir -p ~/l2bot_ws/src
$ cd ~/l2bot_ws/src
```


## 2. Arduino

  1. Download and install the [Arduino IDE](https://www.arduino.cc/en/Main/Software).
  2. Open the IDE and close it once it has finished launching. (This creates all the scketchbook directories on your system)
  3. Add serial write permissions. Run `sudo usermod -a -G dialout <UNAME>` where `<UNAME>` is your username.
  4. Restart Computer
  5. Install ROS Support (see below)

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




## 3. ROS


Clone this repository into the `src` directory.

```
~/l2bot_ws/src$ git clone https://github.com/LTU-AutoEV/l2bot.git
```

Run `catkin_make` from the workspace directory.

```
~/l2bot_ws/src$ cd ..
~/l2bot_ws$ catkin_make
~/l2bot_ws$ source devel/setup.bash
```

Install dependencies

```
~/l2bot_ws$ rosdep install --from-paths src --ignore-src -r -y
```

## 4. Load The Interface onto the Arduino

  1. Open the Arduino IDE
  2. File > Open
    - Open the file `L2Bot_MC.ino` located at `~/l2bot_ws/src/arduino/L2Bot_MC/` 
  3. Plug in the Arduino and turn on the L2Bot
  4. Tools -> Port -> (Select your arduino device)
  5. Upload to arduino


## 5. Identify Your Arduino Board

  1. Plug in your Arduino and turn on the L2Bot
  2. Execute the `detect_arduino.py` script

```
l2bot_ws/src/l2bot$ ./detect_arduino.py
```

# Topics

## Published topics for internal use

  - `std_msgs::UInt16` on **/L2Bot/mc**: Motor controller instruction for the l2bot
