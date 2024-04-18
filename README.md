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







