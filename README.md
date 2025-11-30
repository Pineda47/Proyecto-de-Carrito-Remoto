# Controlador
Para el desarrollo final de la serie de nodos destinados a la comunicación con el Arduino, este nodo se encarga de recibir la información proveniente de los dos nodos a los que está suscrito: el nodo de velocidad y el nodo de control de movimiento.

Su función es agrupar y procesar estos datos para enviar al Arduino las instrucciones que determinan si el vehículo debe retroceder, detenerse o avanzar, de acuerdo con las velocidades generadas por el nodo de velocidad.

El movimiento se mantiene hasta que el nodo de control de movimiento envía un comando de dirección, el cual puede ser A o D, indicando un cambio hacia la izquierda o hacia la derecha respectivamente.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial
import time

# Variables globales
ultimo_modo = "S" # es para que no genere error y es que vaya recto
ultima_velocidad = 1 # que vaya hacia adlente o que el motar gire de forma horaria

# Abrir puerto serial a 115200
arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # esperar a que el Arduino reinicie


def enviar_a_arduino(letra, node):
    try:
        arduino.write(letra.encode())
        node.get_logger().info(f"Enviado por Serial (115200): {letra}")
    except Exception as e:
        node.get_logger().error(f"Error enviando al Arduino: {e}")


def publicar_a_arduino(node):
    global ultimo_modo, ultima_velocidad

    letra = "S"  # una condicion inicial para que no genere error

    if abs(ultima_velocidad) == 0:
        letra = "P"#Parar
    elif ultima_velocidad == 1:
        letra = "S"#delante
    elif ultima_velocidad == -1:
        letra = "B"#retroceder

    # Sobrescribir si viene un modo remoto que sea del caso del nodo controlador_movimento codigo publicador_movimento
    if ultimo_modo in ["A", "D"]:
        letra = ultimo_modo

    node.get_logger().info(f"Enviando letra al Arduino: {letra}")

    # Publicar al tópico ROS
    node.pub_arduino.publish(String(data=letra))

    # Enviar realmente al Arduino por serial
    enviar_a_arduino(letra, node)

def modo_callback(msg, node):
    global ultimo_modo
    ultimo_modo = msg.data
    publicar_a_arduino(node)


#funcion que se conceta al nodo de control_velocidad
def vel_callback(msg, node):
    global ultima_velocidad
    ultima_velocidad = msg.data
    publicar_a_arduino(node)


def main():
    rclpy.init()
    node = Node('coordinador')

    # Publicador
    node.pub_arduino = node.create_publisher(String, '/arduino_comando', 10)

    # Suscriptores
    node.create_subscription(
        String, '/modo', lambda msg: modo_callback(msg, node), 10)
    node.create_subscription(
        Float32, '/vel_motor_a', lambda msg: vel_callback(msg, node), 10)

    node.get_logger().info("Nodo coordinador iniciado (Serial 115200 listo)")
    rclpy.spin(node)

    arduino.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
Mostrado en la terminal de la siguente manera:
``` bash
[INFO] [1764480530.972471635] [coordinador]: Enviado por Serial (115200): W

```


