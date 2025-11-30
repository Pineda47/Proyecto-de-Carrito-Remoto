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
El desarrollo se enfoca en la aplicación y comprensión del lenguaje Python para la creación y funcionamiento de los nodos. Para ello, es necesario crear un topic que sea en Python por medio de la terminal , el cual se estructura de la siguiente manera:
```bash
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp std_msgs
```
Después de esto, se llevan a cabo las dos etapas principales del desarrollo:

1.[Desarrollo previo](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Pruebas/README.md): Consiste en recopilar información y elaborar los esquemas necesarios para la adecuada estructuración del proyecto.

2.[Pruebas](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Pruebas/README.md): En esta etapa se ejecutan, de manera secuencial, los procesos de identificación y verificación para asegurar que cada componente, tanto físico como de código, funcione correctamente.
