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
