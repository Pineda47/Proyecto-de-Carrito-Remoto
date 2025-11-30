# Control remoto vehicular
## Descripcion
El proyecto consiste en el desarrollo de un sistema de publicadores y suscriptores que se comunican de forma serial, con el objetivo de implementar un controlador manual y, al mismo tiempo, un control condicionado por un sensor ultrasónico.
Este sistema permite enviar información hacia el Arduino para controlar los motores, y recibir los datos medidos por el sensor ultrasónico. De esta manera, el vehículo puede operar tanto bajo comando manual como mediante decisiones automáticas basadas en la distancia detectada.


# Desarrollo
El desarrollo se enfoca en la aplicación y comprensión del lenguaje Python para la creación y funcionamiento de los nodos. Para ello, es necesario definir un topic implementado en Python, el cual se estructura de la siguiente manera:
```bash
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp std_msgs
```
luego de entender esa parte se debe realizar estos dos pasas que son mostrados en los siguentes link:
* [Desarrollo-previo]()
* [Pruebas]()



