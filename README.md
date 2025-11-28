# Nodo ultra sonido 
Este nodo se conecta mediante USB, lo que permite verificar si la comunicación en serie entre el Arduino y el PC funciona correctamente. Para ello, se crea una función que determina si la conexión es efectiva. Sin embargo, gran parte de las veces, muchos cables USB no cuentan con los permisos necesarios. Por esta razón, antes de ejecutar el programa, se debe realizar en la PC lo siguiente:
 
 
 sudo chmod 777 /dev/ttyUSB0


```python
import rclpy
from std_msgs.msg import Float32
from rclpy.node import Node
import serial
import time
# variable global
global ardu

# funcion que conecta el puerto usb con la misma frecuencia que el ardiono trabaja para recibir informacion


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
´´´python










[control_velocidad]()
