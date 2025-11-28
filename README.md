# Pruebas de Funcionamiento del Proyecto
El proyecto se validó mediante tres etapas principales, asegurando la correcta comunicación entre nodos, la correspondencia física de los pines y el funcionamiento final del sistema antes de su integración completa.

## Etapa 1: Verificación de Conexiones en Serie entre Nodos

 - Se evaluó la transmisión de datos entre nodos consecutivos, verificando que los valores fueran lógicos y coherentes.

 - Se utilizó un nodo generador de valores de distancia aleatorios y un nodo receptor que determinaba la acción del vehículo:

 - -1: Retroceder

 - 1: Avanzar

- 0: Detenerse

- La prueba se realizó ejecutando ambos programas en terminales separadas, validando la correcta transmisión de datos por USB antes de migrar a Wi-Fi.

## Etapa 2: Análisis de Conexiones Físicas

- Se verificó la correspondencia entre los pines físicos del Arduino y los definidos en el código de PlatformIO.

 - Se ejecutaron programas de prueba para identificar los pines activos y se realizaron alteraciones controladas de un solo pin para observar comportamientos inesperados.

### Objetivo:

- Asegurar que los pines del código correspondan a los pines físicos.

- Garantizar coherencia entre la lógica de programación y las conexiones físicas.

- Minimizar errores al integrar sensores y actuadores.

## Etapa 3: Pruebas Finales

- Antes de construir el vehículo completo, se verificó que todas las conexiones, desde el sensor ultrasónico hasta los motores, funcionaran correctamente al ejecutar los cuatro nodos del sistema.

- Se utilizó un objeto y una regla en el piso para simular distancias reales.

-Con las condiciones de distancia definidas en el código, se comprobó que los motores giraran o se detuvieran según la posición del objeto, validando el funcionamiento integral del sistema.

