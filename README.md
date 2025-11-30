# Control_de_velocidades

En esta parte del proyecto, el nodo está suscrito al nodo encargado de medir distancias. Con base en los valores recibidos, se generan dos posibles salidas que serán publicadas:

1: avanzar hacia adelante

0: detenerse

Para verificar que el publicador del nodo control_velocidad funciona correctamente, se realiza la etapa dos de la rama del método de prueba (si desean ver más detalles, pueden consultar el enlace: [clic aquí](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Pruebas/README.md)
().

Un aspecto importante para el desarrollo de esta sección es determinar una distancia adecuada que permita evitar colisiones y proteger las piezas del vehículo.
Por ello, se define un valor de tipo float, llamado 'umbral', que representa la distancia mínima necesaria para permitir el avance del vehículo. A partir de este parámetro, se aplica una condición dependiente del valor de distancia recibido por el nodo [ultra_sonido]().

A continuacion se presentara el codigo del publicador del control_velcidad con la finalidad de que se pueda replicar y mejorar

def main():
    rclpy.init()
    node = Node('motor_control')

    # Publicador de velocidad
    pub_motor= node.create_publisher(Float32, 'vel_motor', 10)

    # Umbral de distancia
    UMBRAL = 10.0  # cm
    # Callback cuando llega un mensaje de distancia

    def distancia_callback(msg):
        distancia = msg.data
        if distancia >= (UMBRAL):
            velocidad = 1.0  # hacia adelante

        else:
            velocidad = 0.0  # parar

        # Publicar velocidad para determinar si el codigo esta ejecutando bien
        pub_motor.publish(Float32(data=velocidad))
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
## Aclaracion :

## Mejora a futuro:

Se puede implementar una condición adicional que permita que el vehículo retroceda dentro de un intervalo específico. Para ello, se puede introducir un nuevo valor llamado margen, el cual define un rango de distancia entre dos valores.

Por ejemplo, si se define un margen de 5 cm y el umbral es de 15 cm, el intervalo de retroceso sería:

* (15 cm − 5 cm)

Además, la condición inicial debería modificarse para que ya no solo dependa del valor del umbral, sino de una comparación con:

* umbral + margen

De esta forma, el sistema puede tomar decisiones más precisas, permitiendo retroceder cuando la distancia sea demasiado cercana, avanzar cuando sea segura y detenerse en el punto intermedio.

.

