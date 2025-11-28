# Proyecto-de-Carrito-Remoto
Desarrollo de un carrito a control remoto con capacidad de frenar automáticamente al detectar un obstáculo, mediante la integración entre Arduino y Python utilizando comunicación serial en un esquema de publicadores y suscriptores.

## Changes included

- Implementación de un esquema para identificar las diferentes conexiones del Arduino con el sensor ultrasónico y el módulo L298, basado en investigación de fuentes técnicas de cada componente.
- Desarrollo de un mapa serial que identifique qué datos recibe y qué datos envía cada nodo.
- Validación de la coherencia de los pines digitales con los físicos usando PlatformIO (en el archivo main.cpp).
- Desarrollo mínimo de cada nodo, iniciando con valores constantes para observar los resultados que brindaban los otros nodos.
- Identificación de la manera de integrar los nodos Python (ROS2) con Arduino para la comunicación y control del carrito.

## Testing implemented

How the changes were implemented and what is the validation made to them

## Related tickets

Others PRs and Issues considered

## Additional comments

What other information would you like to share?
