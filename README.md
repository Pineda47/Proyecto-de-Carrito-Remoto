# Control_ velocidades
Para esta parte, el nodo está suscrito al nodo de distancias, lo que permite que, a partir de esos valores, se generen tres posibles acciones:

- 1: avanzar hacia adelante.

- -1: retroceder.

- 0: detenerse.

Cada acción se determina con respecto a un intervalo definido por el umbral de distancia. En mi caso, se aplica un umbral de 20 cm, ya que esta distancia permite que la persona pueda reaccionar sin generar dificultades al vehículo.

```python
import rclpy
from std_msgs.msg import Float32
from rclpy.node import Node

#codigo que esta conectado de forma serial 

def main():
    rclpy.init()
    node = Node('motor_a_control')

    # Publicador de velocidad
    pub_motor_a = node.create_publisher(Float32, 'vel_motor_a', 10)

    # Umbral de distancia
    UMBRAL = 20.0  # cm
    margen = 10.0
    # Callback cuando llega un mensaje de distancia

    def distancia_callback(msg):
        distancia = msg.data
        if distancia > (UMBRAL + margen):
            velocidad = 1.0 #hacia adelante
        elif UMBRAL-margen < distancia < UMBRAL+margen:
            velocidad = -1.0 # debe retrocedor
        else:
            velocidad = 0.0 #parar

        # Publicar velocidad para determinar si el codigo esta ejecutando bien
        pub_motor_a.publish(Float32(data=velocidad))
        node.get_logger().info(
            f"Distancia recibida: {distancia:.2f} cm -> Velocidad: {velocidad}")

    # Suscriptor al topic de distancia
    node.create_subscription(
        Float32, 'ultra_distancia', distancia_callback, 10)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
