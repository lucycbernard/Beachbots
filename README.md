# beachbots2023
This is the repository for the BeachBots MQP 2023

## Code Architecture

### small_bot Package
This package contains everything needed for the Smallbot. The launch file is Smallbot.launch.

Smallbot IP: To get the ip address, you will have to connect to the rpi with an ethernet cable and run ifconfig.
This ip address will stay static for a few weeks before changing for an unknown reason. This is something to debug.

## How To Launch Smallbot Code
*For the Smallbot:

username: pi

password: Beachbots2023



### To launch the system:

&emsp; Turn on both Smallbot

&emsp; With your laptop, connect to "WPI-Wireless" wifi
  
&emsp; Open a VNC window using the rpi's ip address
  
&emsp; Enter username (pi) and password (Beachbots2023)
  
&emsp; Enter the following commands
  
  ```
  cd Desktop
  ```
  ```
  sh OpenConnectedROS.sh
  ```
  ```
  roslaunch Chassis.launch
  ```
  ```
  roslaunch Brush.launch
  ```

### To drive the Smallbot:

&emsp; Enter one of the following letters on your keyboard and press enter:

&emsp; S: Stop

&emsp; W: Straight forwards

&emsp; X: Straight backwards

&emsp; A: Left point turn

&emsp; D: Right point turn

&emsp; Q: Left swing turn

&emsp; E: Right swing turn

&emsp; C: 360 turn left

&emsp; B: 360 turn right
