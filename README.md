# Pruebas de Funcionamiento del Proyecto

En esta sección se presentan las pruebas realizadas para validar el funcionamiento del proyecto. Se recomienda seguir ambas etapas si se desea replicar el proyecto, utilizando los códigos de prueba incluidos.

Etapa 1: Verificación de Conexiones en Serie entre Nodos

En esta etapa se enfocó en verificar las conexiones en serie entre los nodos, evaluando de manera secuencial que los valores transmitidos sean lógicos y coherentes entre cada par de nodos.

El procedimiento se realizó de la siguiente manera:

Prueba de comunicación entre nodos consecutivos:
Se aplicó la verificación de dos en dos nodos para asegurar que los datos transmitidos fueran consistentes y correctos.

Caso específico – Nodo generador de distancias y nodo receptor:

El primer nodo genera valores de distancia aleatorios para simular la lectura de un sensor ultrasónico.

En el setup se definió el nombre del publicador al que debe conectarse el siguiente nodo.

Para fines de análisis, los valores recibidos se muestran en la terminal mediante println(valor de la distancia), permitiendo verificar que la conexión USB funciona de manera óptima antes de implementar la comunicación Wi-Fi.

Integración con el nodo de control de velocidad:

Una vez confirmada la comunicación entre el nodo generador de distancias y el nodo receptor, se conecta el suscriptor de control de velocidad.

Este nodo recibe los valores de distancia y determina la acción del vehículo según la siguiente regla:

-1: Retroceder

1: Avanzar

0: Detenerse

Ejecución de la prueba:

Para realizar la evaluación correctamente, es necesario ejecutar ambos programas en dos terminales diferentes: uno para el nodo que genera los valores de distancia y otro para el nodo de control de velocidad.

Se espera que la terminal del suscriptor muestre las acciones correspondientes a los valores recibidos, verificando así la correcta transmisión de datos.
