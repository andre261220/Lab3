#!/usr/bin/env python

import rospy
from turtlesim.srv import Spawn

def spawn_turtle(x, y, name):
    rospy.wait_for_service('/spawn')
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, 0, name)
        rospy.loginfo("Tortuga spawneda exitosamente con nombre: %s", name)
    except rospy.ServiceException as e:
        rospy.logerr("Error al spawnear la tortuga: %s", e)

if __name__ == '__main__':
    rospy.init_node('spawn_turtle_node')
    
    x_pos = float(input("Ingrese la posición x inicial de la tortuga: "))
    y_pos = float(input("Ingrese la posición y inicial de la tortuga: "))
    turtle_name = input("Ingrese el nombre de la tortuga: ")
    
    spawn_turtle(x_pos, y_pos, turtle_name)
