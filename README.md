Startreihenfolge

Treiber für youBot:
1. sudo udevadm trigger
2. roslaunch oberstufenlabor_mechatronik_robot base_system.launch

Für fertiges Programm:
1. roslaunch oberstufenlabor_mechatronik_robot vision.launch
2. roslaunch oberstufenlabor_mechatronik_robot collector_server.launch
3. rosrun oberstufenlabor_mechatronik_robot exercise04_collector_node Color number

Connect to Robot:
ssh -X imesadmin@192.168.1.230
Passwort: youbot
