# Nodo control de movimento
Este nodo es independiente de los demás, ya que se caracteriza por interactuar en tiempo real con el usuario. Su función es permitir que la persona ingrese una letra, la cual será almacenada y enviada posteriormente al último nodo.

El publicador solicitará al usuario que digite una letra que indique la dirección deseada:

- W: avanzar recto, sin alterar el curso.

- A: girar a la derecha.

- D: girar a la izquierda.

- S: Parar
  Son  el mismo sistema para algunos videjuegos.
El propósito de este nodo es permitir que el usuario pueda modificar la dirección mientras el vehículo, mediante el nodo 2 de control de velocidades, determina si debe detenerse, retroceder o continuar avanzando según los valores de distancia recibidos.El desarrollo de la prueba para el codigo se presenta en al etapa 3 que se explica en el rama de pruebas [click aqui].
## Concepto de mejora futura:
Posible desarrollo de otro nodo que tenga la capacidad de generar la potenica que requiere girar el motar como lo de los carros (1,2,3,4,R).


```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading


def leer_terminal(node, pub):
    while rclpy.ok():
        # Convertir a mayúscula por si acaso
        # siendo A=izquierda , S=recto y D= Derecha
        comando = input("Ingresa A, S ,D: ").upper()
        if comando not in ["A", "S", "D", "W"]:
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

    print("Nodo de control remoto iniciado. Ingresa A, S o D, W por terminal.")

    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
