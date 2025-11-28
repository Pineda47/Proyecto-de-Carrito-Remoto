# Pruebas de Funcionamiento del Proyecto

En esta sección se presentan las pruebas realizadas para validar el funcionamiento del proyecto. Se recomienda seguir ambas etapas si se desea replicar el proyecto, utilizando los códigos de prueba incluidos.

## Etapa 1: Computacional

En esta etapa se analiza si las interacciones entre nodos están funcionando correctamente. Se inicia con el nodo de `ultra_sonido`.

El objetivo de esta prueba es generar y publicar valores aleatorios de tipo `Int32` en ROS2, hasta alcanzar un total de 10 números.

### Código de Prueba

import rclpy
from std_msgs.msg import Int32
import numpy as np
import time

def main():
    # Inicializar ROS2
    rclpy.init()
    node = rclpy.create_node('random_int_node')

    # Crear publicador de tipo Int32
    pub = node.create_publisher(Int32, 'numeros_aleatorios', 10)

    # Generar y publicar 10 números aleatorios
    for i in range(10):
        num = np.random.randint(0, 100, dtype=np.int32)
        msg = Int32()
        msg.data = int(num)  # Convertir a Python int
        pub.publish(msg)
        node.get_logger().info(f"Número publicado: {msg.data}")
        time.sleep(1)  # Espera 1 segundo entre publicaciones

    node.get_logger().info("Se publicaron los 10 números. Nodo apagándose...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

