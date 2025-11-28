# Proyecto-de-Carrito-Remoto
Desarrollo de un carrito a control remoto con capacidad de frenar automáticamente al detectar un obstáculo, mediante la integración entre Arduino y Python utilizando comunicación serial en un esquema de publicadores y suscriptores.

## Changes included
- Implementación de un esquema para identificar las diferentes conexiones del Arduino con el sensor ultrasónico y el módulo L298, basado en investigación de fuentes técnicas de cada componente. El esquema puede visualizarse y explicarse en una de las ramas presentes: [Click aquí](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/tree/Desarrollo-previo).  
Asimismo, se desarrolló un mapa serial que identifica qué datos recibe y qué datos envía cada nodo, permitiendo verificar el tipo de valores enviados (float32 y strings). Esto facilita la comprensión de la comunicación entre nodos y Arduino en el sistema.

- Validación de la coherencia de los pines digitales con los físicos usando PlatformIO (archivo `main.cpp`) mediante pruebas para confirmar que los pines corresponden correctamente a cada función.  
- Desarrollo mínimo de cada nodo, iniciando con valores constantes (como velocidades fijas) para observar el comportamiento de los otros nodos y la respuesta del sensor ultrasónico en el Arduino (`main.cpp`).  
- Identificación de la manera de integrar los nodos Python (ROS2) con Arduino para la comunicación y control del carrito, desarrollada en dos etapas que serán presentadas en la rama de análisis final.

## Testing implemented

Las pruebas se realizaron en dos etapas:  

1. **Computacional:** Se aplicaron valores fijos para observar cómo reaccionaban los demás nodos.  
2. **Práctica / Física:** Se ensamblaron los componentes uno a uno para verificar la correcta conexión y funcionamiento de cada pieza.  
- Esta fase está documentada en la rama de experimentación. [Click aquí]([Pruebas](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/tree/Pruebas)).

## Related tickets

Se enfocó en identificar las tareas y objetivos principales del proyecto:  
- Verificar las conexiones de cada componente al Arduino.  
- Comprobar que el sensor ultrasónico brinde distancias coherentes.  
- Confirmar que los motores conectados al L298 funcionen correctamente sin la intervención del Arduino.  
- Asegurar que los motores puedan variar su velocidad y dirección según los comandos recibidos.  
- Garantizar una conexión estable entre Arduino y PC.  
- Confirmar que la comunicación entre nodos sea serial.  
- Permitir enviar comandos de dirección desde la terminal.

## codigos de trabajo.
Acontuniacion se presntaran los nodos con sus nombres el nombre para ejecutarlos y los valores que debeb mostrar en la terminal 




## Additional comments

