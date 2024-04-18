# Lab3
Distancias euclideanas con ROS-Diseño de Sistemas Robóticos 


Una operación común en robótica y procesamiento de datos espaciales es calcular la distancia entre dos puntos en un espacio Euclidiano. La distancia euclidiana es la longitud de la línea recta que une dos puntos en un espacio Euclidiano. En un plano bidimensional, la fórmula para calcular la distancia euclidiana entre dos puntos (x1, y1) y (x2, y2) es la raíz cuadrada de la suma de los cuadrados de las diferencias en las coordenadas x e y.

En esta practica, creamos un nodo de ROS en Python que genere dos puntos aleatorios en el espacio 2D, calcule la distancia euclidiana entre ellos y dibuje una línea entre los dos puntos en Turtlesim para visualizar la distancia calculada.

## Problemas

![Distancias euclideanas](https://github.com/andre261220/Lab3/assets/132303647/755d8c6c-2358-490c-84e4-bc90c35eb4bb)

1. Calcular y mostrar en pantalla la DTG y ATG.
2. No mover el robot, hacer spawn del mismo en la posición Goal.
3. Explicar el mapeo necesario para las velocidades.
4. Usar un controlador (libre) para llevar a la tortuga a la posición deseada, hacerlo en bucleinfinito.

A continuacion se muestras los resultados obtenidos de la presente practica, donde primero tenemos que diseñar un codigo que mata cualquier instancia existente del robot en la posición "Goal", si es que existe, y luego spawnear el nuevo robot en la misma posición. Esto asegurará que no haya ninguna interferencia o movimiento involuntario antes de que el nuevo robot sea spawnado.

Entonces como quedamos el presente codigo crea una nueva instancia de una tortuga en el simulador Turtlesim, el cual antes de crear una nueva tortuga, primero mata a la tortuga existente llamada "turtle1" antes de spawnear una nueva tortuga en la posición y con los nombres nuevos proporcionados. Si está presente, aqui necesitamos tener tres argumentos: la posición x y y donde se desea spawnear la tortuga, y el nombre de la tortuga, luego indicaremos que la tortuga fue spawneda exitosamente con el nombre proporcionado, el codigo es el siguiente:

## Codigo para matar "turtle1" y crear una nueva (spawn.py)
```python
#!/usr/bin/env python

import rospy
from turtlesim.srv import Spawn, Kill

def spawn_turtle(x, y, name):
   rospy.wait_for_service('/spawn')
   try:

       kill = rospy.ServiceProxy('/kill', Kill)
       kill("turtle1")  # Mata la tortuga anterior

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
```

Ahora en el caso para hacer el spawn en el turtleSim necesitamos crear un nodo en ROS que permita al usuario spawnear una nueva tortuga en el simulador proporcionando la posición inicial (coordenadas X y Y) y un nombre para la tortuga.

## codigo para spawnear la tortuga (nturtle.py):
```python
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
```

Lo que sucede al ejecuta el script, se solicita al usuario que ingrese la posición X y Y inicial de la tortuga, así como el nombre que desea asignarle, luego llama a la función spawn_turtle con estas entradas proporcionadas por el usuario. Dentro de la función spawn_turtle, se espera a que el servicio /spawn esté disponible, luego se crea un proxy de servicio para /spawn y se llama al servicio con los datos proporcionados por el usuario (posición X, Y y nombre de la tortuga). Si la operación se realiza con éxito, se registra un mensaje informativo. En caso de que ocurra un error, se registra un mensaje de error.

Al ejecutar primero el codigo de "spawn.py" y luego el "nturtle.py" garantiza que cualquier instancia existente de la tortuga llamada "turtle1" sea eliminada antes de crear una nueva tortuga en la simulación. Esto asegura que el "spawn.py" pueda crear la tortuga en la posición deseada sin ninguna interferencia de instancias previas y asi obtener lo siguiente: 

PONER IMAGEN 

Ahora para el mapeo de velocidades se realiza de manera implícita en la comunicación con el simulador Turtlesim a través de ROS. En ambos codigos se utilizan funciones proporcionadas por ROS y el paquete Turtlesim para manejar la creación y el control de las tortugas en el simulador. El mapeo de velocidades se realiza en el contexto de ROS y Turtlesim, y es importante comprender cómo estas herramientas gestionan las velocidades de las tortugas.

Para el "spawn.py":

El mapeo de velocidades se realiza internamente por Turtlesim cuando se invoca la función spawn para crear una nueva tortuga, esta función toma como argumentos la posición x y y, así como el nombre de la tortuga, Turtlesim se encarga de traducir estas coordenadas y el nombre de la tortuga en comandos específicos para su simulación. Esto implica la traducción de coordenadas cartesianas a coordenadas en el espacio del simulador y la asignación de un identificador único para la nueva tortuga.

Para "nturtle.py":

En el caso antes de spawnear una nueva tortuga, se mata cualquier instancia existente de la tortuga llamada "turtle1" utilizando la función kill, esto asegura que no haya conflictos con la nueva tortuga que se está por crear.

Después de eliminar la tortuga existente, se utiliza la función spawn para crear una nueva tortuga en la posición especificada por el usuario y al igual que en el "spawn.py", Turtlesim se encarga del mapeo de las coordenadas y el nombre de la tortuga en comandos de simulación adecuados. En ambos casos, el mapeo de velocidades se realiza de manera transparente para el usuario a través de las funciones proporcionadas por ROS y Turtlesim. El usuario simplemente proporciona las coordenadas y el nombre de la tortuga, y el sistema se encarga de traducir estas entradas en comandos específicos para el simulador. Esto demuestra cómo ROS facilita el desarrollo de aplicaciones robóticas al abstraer gran parte de la complejidad del control del hardware y la simulación.

Para el paso 4 necesitamos usar un controlador (libre) para llevar a la tortuga a la posición deseada, hacerlo en bucle infinito, en este caso nosotros estamos usando un controlador PID (Proporcional-Integral-Derivativo) para mover una tortuga en el simulador Turtlesim hacia una posición y orientación deseadas.

Controlador PID:

Utiliza un controlador PID para calcular las velocidades lineales y angulares necesarias para mover la tortuga hacia una posición y orientación específicas, este utiliza la retroalimentación de la posición actual de la tortuga para calcular los errores de posición y orientación, y ajusta continuamente el movimiento de la tortuga para minimizar estos errores.

para su funcionamiento primero suscribe al topic /turtle1/pose para obtener la posición y orientación actual de la tortuga en el simulador, despues publica en el topic /turtle1/cmd_vel para enviar comandos de velocidad a la tortuga y controlar su movimiento, asi permite al usuario ingresar manualmente la posición y orientación deseadas para la tortuga.

Se utiliza un bucle infinito para mover continuamente la tortuga hacia la posición y orientación deseadas hasta que se interrumpa manualmente el programa para proporcionar un controlador automático para mover la tortuga a posiciones y orientaciones específicas en el simulador Turtlesim. En este caso el bucle se encuentra dentro del método move_turtle_to_desired_pose. Este método se encarga de calcular y aplicar las velocidades necesarias para mover la tortuga a la posición y orientación deseadas, y se ejecuta en un bucle infinito hasta que se alcance la posición deseada.

## codigo del bucle 
```python
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
            error_theta = atan2(error_y, error_x) - self.current_theta
            
            # Normalizar el ángulo
            if error_theta > radians(180):
                error_theta -= radians(360)
            elif error_theta < -radians(180):
                error_theta += radians(360)
            
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

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la pose deseada del usuario
            desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()

            # Mover la tortuga a la posición deseada
            self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)

    def get_desired_pose_from_user(self):
        print("Ingrese la posición y orientación deseadas:")
        desired_x = float(input("Coordenada x: "))
        desired_y = float(input("Coordenada y: "))
        desired_theta = radians(float(input("Orientación (en grados): ")))
        return desired_x, desired_y, desired_theta

if __name__ == '__main__':
    try:
        move_turtle_pid = MoveTurtlePIDControl()
        move_turtle_pid.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
```
Explicación del bucle:

* El bucle while not rospy.is_shutdown(): se ejecuta continuamente mientras ROS no esté en el proceso de apagado (es decir, mientras el nodo esté en funcionamiento).
* Dentro del bucle, se calculan los errores de posición y orientación entre la posición actual y la deseada.
* Luego, se utilizan estos errores para calcular las velocidades lineales y angulares del movimiento utilizando el controlador PID.
* Se crea un mensaje de tipo Twist con estas velocidades calculadas y se publica en el topic /turtle1/cmd_vel.
* Se imprime la posición actual y los errores en la terminal para fines de depuración.
* Se verifica si se alcanza la posición deseada. Si es así, se registra un mensaje informativo y se rompe el bucle.
* Finalmente, se espera hasta la siguiente iteración utilizando self.rate.sleep(), lo que asegura que el bucle se ejecute a una tasa específica (en este caso, 10 Hz.






