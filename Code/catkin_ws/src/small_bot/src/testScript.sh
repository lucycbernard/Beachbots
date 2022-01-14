echo "starting project execution"

#gnome-terminal -- ./runRosMaster.sh
gnome-terminal -- roscore

sleep 5s

gnome-terminal -- rosrun small_bot talker.py

gnome-terminal -- rosrun small_bot listener.py

gnome-terminal -- rostopic echo /chatter

# gnome-terminal -x bash -c "
#     echo \"hello 1\" && 
#     echo \"hello 2\" && 
#     read varName &&
#     echo hello $varName &&
#     read waitHere
#     "
