# Desarrollo previo

En esta sección se enfoca en la identificación de las características de cada componente requerido para el desarrollo del sistema del vehículo.

# Arduino

El ESP8266 es un microcontrolador con capacidad de conexión Wi-Fi, y entre sus principales características se encuentran:

- Wi-Fi integrado: permite conectarse a redes inalámbricas para enviar y recibir datos.
- GPIOs (pines de entrada/salida): se pueden utilizar para controlar motores y sensores.
- Compatibilidad con PlatformIO: se puede programar de manera similar a un Arduino tradicional.
- Bajo consumo de energía: ideal para proyectos que funcionan con baterías.
-Limitación de comunicación: los Arduinos solo pueden recibir un dato y enviar un dato a la vez; no es posible recibir o transmitir dos o más valores simultáneamente.

![Conexión Arduino](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Desarrollo-previo/coneccion%20arduino.jpeg)

# L298N

El L298N** es un driver o controlador de motores DC y motores paso a paso**.  
Permite que un microcontrolador (como Arduino o ESP8266) controle motores sin que la corriente del motor dañe al microcontrolador.



## Características principales

- Control bidireccional de 2 motores DC o 1 motor paso a paso**.  
- Tensión de operación: hasta 46 V.  
- Corriente máxima por canal: 2 A (requiere disipador de calor para altas corrientes).  
- Entradas de control: IN1, IN2, IN3, IN4 (para definir dirección y movimiento de los motores).  
- Salida PWM: permite controlar la velocidad de los motores mediante modulación por ancho de pulso (PWM).  
- Protección térmica:** evita daños por sobrecalentamiento; puede requerir disipador de calor.  



## Funcionamiento básico

- Para controlar un motor, conecta los pines IN1, IN2 y ENA al microcontrolador.  
- Para controlar el segundo motor, usa IN3, IN4 y ENB.  
- Ajustando los pines de entrada y la señal PWM, puedes:  
  - Girar el motor hacia adelante.  
  - Girar el motor hacia atrás.  
  - Detener el motor.
    
![Conexiones-L298N](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/Desarrollo-previo/Conexiones-L298N.png)

# Sensor ultrasónico HC-SR04

El HC-SR04 es un sensor de distancia que mide la distancia a un objeto usando ondas ultrasónicas. Es muy popular en proyectos de robótica, drones y sistemas de detección de obstáculos.

## Características principales

- Rango de medición: 2 cm a 400 cm aproximadamente.
- Precisión: ±3 mm.
- Voltaje de operación: 5 V.

## Pines de conexión

- VCC: alimentación positiva.
- GND: tierra.
- TRIG: pin de disparo; se envía un pulso para emitir el sonido.
- ECHO: pin de recepción; se recibe el pulso reflejado para calcular la distancia.

## Principio de funcionamiento

1. Se envía un pulso ultrasónico desde el pin TRIG.
2. El pulso rebota en un objeto y regresa al sensor.
3. El pin ECHO mide el tiempo que tardó en regresar.
4. Con el tiempo y la velocidad del sonido, se calcula la distancia al objeto.
