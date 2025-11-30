# Pruebas de Funcionamiento del Proyecto

El proyecto fue validado mediante varias etapas, asegurando la correcta comunicación entre nodos, la correspondencia física de los pines y el funcionamiento completo del sistema antes de su integración final.

## 1. Funcionamiento del sensor ultrasónico

En esta etapa se emplea el nodo de ultra_sonido junto con el código de Arduino para verificar su comportamiento. Mediante una regla y un objeto de prueba —en este caso, un carrito de juguete— se comprueba que los pines estén colocados correctamente y que el Arduino reciba respuesta del sensor.

Uno de los métodos clásicos para confirmar que el sensor ultrasónico está encendido es acercar el oído al dispositivo y escuchar un ligero sonido emitido por el movimiento de sus transductores. Este ruido indica que el sensor está operando adecuadamente.

A continuación, se presentan imágenes que muestran las pruebas realizadas.

![](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Pruebas/primera%20prueba.jpeg)

y en la terminal debe salir:
bash
distancia 18 cm
## 2. Coneccion entre dos nodos
Para esta etapa se utilizan los resultados obtenidos en la primera fase, ejecutando simultáneamente el nodo de ultra_sonido
 y el nodo de velocidades.Para ello, se abren dos terminales:

 * En la primera se ejecuta el nodo de medición de distancia.

 * En la segunda se ejecuta el nodo encargado del [control de velocidad](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Control-de-velocidades/README.md).

A partir de la distancia publicada por el nodo de ultrasonido, el nodo de velocidades debe generar una de las tres posibles salidas:

1: avanzar

0: detenerse

-1: retroceder

En el caso del ejemplo presentado anteriormente, la distancia medida corresponde a un valor seguro, por lo que la salida esperada es:

velocidad = 1
## 3. Coneccion del trecer nodo
En el desarrollo de esta etapa se separa ligeramente el flujo de las pruebas anteriores. Para ello, se ejecuta en una terminal adicional el nodo de control_de_movimiento.
El objetivo es verificar que este nodo esté publicando correctamente los comandos y que responda adecuadamente a las instrucciones ingresadas desde la terminal.

En esta prueba, el nodo presenta un enunciado solicitando al usuario seleccionar una de las letras habilitadas. Cada letra corresponde a una acción específica del sistema, lo que permite comprobar que los mensajes se publiquen correctamente.

## 4. Desarrollo de los nodos de forma serial 
En esta parte se aplica todo los nods en cutro terminales diferentes ejecutando primero el de ultra sonido luego el de velocidades , despues el de control de movimentos  y al funal el de controldor para determinar que si el ultimo publique en la pantalla las direcciones que va a desarrollar que esta condicionada por el comando y por la distancia  brindada anteiormente.

## 5. Desarrollo final:
esta es la ultima etapa que consiste en la coneccion del vechiculo por medio de una usb y deterinar si el funcionamento del vehiculo.

