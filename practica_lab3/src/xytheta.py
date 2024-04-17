#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, radians, sqrt

class MoveTurtlePIDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_pid')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Variables para el controlador PID en cada eje
        self.Kp_x = 1
        self.Kp_y = 1
        self.Kp_theta = 1

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_pose(self, desired_x, desired_y, desired_theta):
        while not rospy.is_shutdown():
            # Calcular los errores de posición
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            error_theta = desired_theta - self.current_theta
            
            # Calcular las velocidades lineales y angular del movimiento
            vel_x = self.Kp_x * error_x
            vel_y = self.Kp_y * error_y
            vel_theta = self.Kp_theta * error_theta
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            twist_msg.angular.z = vel_theta
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual y los errores en la terminal
            rospy.loginfo("Posición actual: x=%f, y=%f, theta=%f", self.current_x, self.current_y, self.current_theta)
            rospy.loginfo("Errores: ex=%f, ey=%f, etheta=%f", error_x, error_y, error_theta)
            
            # Verificar si se alcanza la posición deseada
            if sqrt(error_x**2 + error_y**2) < 0.1 and abs(error_theta) < radians(5):
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

    def get_desired_pose_from_user(self):
        print("Ingrese la posición y orientación deseadas:")
        desired_x = float(input("Coordenada x: "))
        desired_y = float(input("Coordenada y: "))
        desired_theta = radians(float(input("Orientación (en grados): ")))
        return desired_x, desired_y, desired_theta

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la pose deseada del usuario
            desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()

            # Mover la tortuga primero en x
            self.move_turtle_to_desired_pose(desired_x, self.current_y, self.current_theta)
            
            # Mover la tortuga luego en y
            self.move_turtle_to_desired_pose(desired_x, desired_y, self.current_theta)

            # Mover la tortuga finalmente en theta
            self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_pid = MoveTurtlePIDControl()
        move_turtle_pid.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
