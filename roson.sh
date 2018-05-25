echo hola
##cd /home/local/ITAM/acruzyv/practica_2/pract_2_ros
#rosnode kill serial_node
#rosnode kill matplotlib_gui
source /home/local/ITAM/acruzyv/practica_2/pract_2_ros/devel/setup.sh
rosrun rosserial_python serial_node.py /dev/ttyACM0&
source /home/local/ITAM/acruzyv/Arduino/AutoNOMOS-Tools/tutorial-gui/devel/setup.sh
rosrun matplotlib_gui plot.py
rosnode kill serial_node
#rosnode kill matplotlib_gui
