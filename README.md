# beachbots2023
This is the repository for the BeachBots MQP 2023

## Code Architecture

### base_bot Package
This is where all the Basebot code is. The launch file is DetectApriltags.launch. 

Basebot IP: 192.168.0.10

### small_bot Package
This package contains everything needed for the smallbot. The launch file is Smallbot.launch.

Smallbot IP: 192.168.0.11

## How To Launch Basebot and Smallbot Code
*For both the Smallbot and Basebot:

username: pi

password: raspberry



### To launch the system:

&emsp; Turn on both Smallbot and Basebot

&emsp; With your laptop, connect to "BasebotNetwork" wifi with the password "Wumpus3742"
  
&emsp; In Putty, ssh into the Basebot (192.168.0.10)
  
&emsp; In the terminal, enter username (pi) and password (raspberry)
  
&emsp; Enter the following commands
  
  ```
  cd Desktop
  ```
  ```
  sh OpenConnectedROS.sh
  ```
  ```
  roslaunch DetectAprilTags.launch
  ```
  
&emsp; In second Putty window, ssh into the Smallbot (SmallBot.local) (192.168.0.11)
  
&emsp; In the terminal, enter username (pi) and password (raspberry)
  
&emsp; Enter the following commands
  
  ```
  cd Desktop
  ```
  ```
  sh OpenConnectedROS.sh
  ```
  ```
  roslaunch Smallbot.launch
  ```
  
### Debugging
To confirm that the Basebot is functioning and communicating with the Smallbot:

In new Putty window with Basebot and Smallbot running, ssh into the Smallbot (192.168.0.11)

In the terminal, enter username (pi) and password (raspberry)

Enter the following command

  ```
  cd Desktop
  ```
  ```
  sh OpenConnectedROS.sh
  ```
  ```
  rostopic echo /tag_bounds/tag_3
  ```  
This will print the bounds of the AprilTag:

  -1 = left
  
  0 = center
  
  1 = right
