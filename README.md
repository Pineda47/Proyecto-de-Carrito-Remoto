# Control remoto vehicular
## Descripcion
El proyecto consiste en el desarrollo de un sistema de publicadores y suscriptores que se comunican de forma serial, con el objetivo de implementar un controlador manual y, al mismo tiempo, un control condicionado por un sensor ultrasónico.
Este sistema permite enviar información hacia el Arduino para controlar los motores, y recibir los datos medidos por el sensor ultrasónico. De esta manera, el vehículo puede operar tanto bajo comando manual como mediante decisiones automáticas basadas en la distancia detectada.

## Objetivos
1. Generar una configracion serial.
2. Presentar una coneccion entre el computaodr y el adruino.
3. Entender el funcionamiento de cada componenete fisico del vehiculo.
4. gnerar dependiandoa de movimento del vehciluclo al sensor ultra sonido
   ## Desarrollo
El desarrollo se enfoca en la aplicación y comprensión del lenguaje Python para la creación y funcionamiento de los nodos. Para ello, es necesario definir un topic implementado en Python, el cual se estructura de la siguiente manera:
```bash
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp std_msgs
```
luego de entender esa parte se debe realizar estos dos pasas que son mostrados en los siguentes link:
* [Desarrollo-previo](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Desarrollo-previo/README.md)
* [Pruebas](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Pruebas/README.md)
## Aplicaciones futuras 
Se puede comprender que, si se logra un desarrollo efectivo de la relación entre el sensor y el movimiento del vehículo, esta tecnología puede aplicarse al diseño mecánico y tecnológico de nuevos vehículos. Su implementación permitiría incrementar la seguridad vial, ya que brindaría apoyo a los conductores en situaciones donde puedan estar distraídos o no perciban correctamente los obstáculos en la vía.
## codigos de desarrollo:
1. [ultra sonido](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Ultra-sonido/README.md)
2. [Control de velocidades](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Control-de-velocidades/README.md)
3. [Control de movimento](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/control-de-movimento/README.md)
4. [Controlador](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/control-de-movimento/README.md)
5. [Arduino](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Arduino/README.md)
## Autor 
Juan Camilo Pienda


