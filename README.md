
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading


def leer_terminal(node, pub):
    while rclpy.ok():
        # Convertir a mayúscula por si acaso
        # siendo A=izquierda , S=recto y D= Derecha
        comando = input("Ingresa A, S ,D: ").upper()
        if comando not in ["A", "S", "D"]:
            print("Valor inválido, intenta de nuevo.")
            continue
        pub.publish(String(data=comando))
        print(f"Comando publicado: {comando}")


def main():
    rclpy.init()
    nodo = Node("control_remoto")

    # Publicador al tópico /modo como String
    pub = nodo.create_publisher(String, '/modo', 10)

    # Hilo para leer terminal sin bloquear ROS
    hilo = threading.Thread(target=leer_terminal, args=(nodo, pub))
    hilo.daemon = True
    hilo.start()

    print("Nodo de control remoto iniciado. Ingresa A, S o D por terminal.")

    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
