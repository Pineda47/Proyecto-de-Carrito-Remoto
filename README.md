# Proyecto-de-Carrito-Remoto
Desarrollo de un carrito a control remoto con capacidad de frenar automáticamente al detectar un obstáculo, mediante la integración entre Arduino y Python utilizando comunicación serial en un esquema de publicadores y suscriptores.

## Changes included
- Implementación de un esquema para identificar las diferentes conexiones del Arduino con el sensor ultrasónico y el módulo L298, basado en investigación de fuentes técnicas de cada componente. El esquema puede visualizarse y explicarse en una de las ramas presentes: [Click aquí](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/tree/Desarrollo-previo).  
Asimismo, se desarrolló un mapa serial que identifica qué datos recibe y qué datos envía cada nodo, permitiendo verificar el tipo de valores enviados (float32 y strings). Esto facilita la comprensión de la comunicación entre nodos y Arduino en el sistema.

- Validación de la coherencia de los pines digitales con los físicos usando PlatformIO (archivo `main.cpp`) mediante pruebas para confirmar que los pines corresponden correctamente a cada función.  
- Desarrollo mínimo de cada nodo, iniciando con valores constantes (como velocidades fijas) para observar el comportamiento de los otros nodos y la respuesta del sensor ultrasónico en el Arduino (`main.cpp`).  
- Identificación de la manera de integrar los nodos Python (ROS2) con Arduino para la comunicación y control del carrito, desarrollada en dos etapas que serán presentadas en la rama de análisis final.

## Testing implemented

Las pruebas se realizaron en dos etapas:  

1. **Computacional:** Se aplicaron valores fijos para observar cómo reaccionaban los demás nodos.  
2. **Práctica / Física:** Se ensamblaron los componentes uno a uno para verificar la correcta conexión y funcionamiento de cada pieza.  
- Esta fase está documentada en la rama de experimentación. [Click aquí](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/tree/Pruebas).

## Related tickets

Se enfocó en identificar las tareas y objetivos principales del proyecto:  
- Verificar las conexiones de cada componente al Arduino.  
- Comprobar que el sensor ultrasónico brinde distancias coherentes.  
- Confirmar que los motores conectados al L298 funcionen correctamente sin la intervención del Arduino.  
- Asegurar que los motores puedan variar su velocidad y dirección según los comandos recibidos.  
- Garantizar una conexión estable entre Arduino y PC.  
- Confirmar que la comunicación entre nodos sea serial.  
- Permitir enviar comandos de dirección desde la terminal.

## codigos de trabajo.
Acontuniacion se presntaran los nodos con sus nombres el nombre para ejecutarlos y los valores que debeb mostrar en la terminal :
Como se menciona en el desarrollo previo se segira el mismo esquema
### Nodo de ultra_sonido
El nodo tiene como nombre"ultra_sonido" y se creo dentro de un un subscritor que recibe la unformacion por via USB las distancias que brinda el ultrasonido para que lego las muestre :

```python
import rclpy
from std_msgs.msg import Float32
from rclpy.node import Node
import serial
import time

# variable global
global ardu

# función que conecta el puerto USB con la misma frecuencia que el Arduino trabaja para recibir información
def create_arduino_connection(node: Node):
    port = node.get_parameter("port").value
    baudRate = node.get_parameter("baudRate").value
    try:
        ser = serial.Serial(port, baudRate, timeout=1)
        time.sleep(2)  # Espera a que el Arduino se reinicie
        node.get_logger().info(
            f"Conectado a Arduino en {port} a {baudRate} baud")
        return ser
    except serial.SerialException as e:
        node.get_logger().error(f"No se pudo abrir el puerto {port}: {e}")
        rclpy.shutdown()
        exit(1)


def main():
    global ardu

    rclpy.init()
    node = rclpy.create_node('ultra_distancia_node')

    # Parámetros del puerto
    node.declare_parameter("port", "/dev/ttyUSB0")
    node.declare_parameter("baudRate", 115200)

    # Publicador ROS2
    pub = node.create_publisher(Float32, 'ultra_distancia', 10)

    # Conectar con Arduino
    ardu = create_arduino_connection(node)

    try:
        while rclpy.ok():
            if ardu.in_waiting > 0:
                linea = ardu.readline().decode('utf-8', errors='ignore').strip()
                if linea:
                    try:
                        distancia = float(linea)
                        pub.publish(Float32(data=distancia))
                        node.get_logger().info(
                            f"Distancia publicada: {distancia:.2f} cm")
                    except ValueError:
                        node.get_logger().warn(f"Valor no válido: {linea}")
    except KeyboardInterrupt:
        pass

    ardu.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
-----------------------------------
```python

En el el setup.py del nodo se debe escriber el nombre de efecucion que seria de la siguente manera:
from setuptools import find_packages, setup

package_name = 'ultra_sonido'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juan-camilo-pineda',
    maintainer_email='juan.piendah@javeriana.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscritor_ultra_sonido = ultra_sonido.subscritor_ultra_sonido:main',
        ],
    },
)


