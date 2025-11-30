# Arduino 
Para el desarrollo y la planificación del sistema, es necesario cargar mediante USB el código base de Arduino. Este código debe tener la capacidad de recibir los valores enviados por el nodo final y procesarlos correctamente.

En este caso, los motores deben girar en direcciones opuestas cuando se ordena un giro, es decir, cada motor debe activar sus pines de dirección en sentido contrario para generar el efecto de rotación. Por el contrario, cuando el vehículo avanza o retrocede, ambos motores deben mantener la misma configuración de sentido.

En esta etapa se identifican y se caracterizan las ubicaciones de cada pin:

- Motor 1: pines D5, D6 y D7.

- Motor 2: pines D3, D4 y D0.

- Sensor ultrasónico: pines D1 y D2.

Además, en esta parte se realiza el cálculo para determinar la distancia que será recibida por el nodo 1, correspondiente al procesamiento del sensor ultrasónico.


```python
#include <Arduino.h>

// pines para el motor 1 que son los mismo que estan en fisico
const int IN1 = D5;
const int IN2 = D6;
const int ENA = D7; // velocidad motor 1

// pines para el motor 2 que solo se requiere que se apague o se prenda nada mas
const int IN3 = D3;  // direccion motor 2
const int IN4 = D4; // segundo pin de direccion motor 2 (nuevo)
const int ENB = D0;  // velocidad motor 2 igual que el motor A (PWM)

// pines para el snesor ultrasonido
const int TRIG = D1; // para este caso se usa D3 en la parte fisica
const int ECHO = D2; // para este caso se usa D4 en la parte fisica

// funcion para medir la distancia del ultrasonido
long medirDistancia()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10); // tiempo que tarda en registrar los valores de las distancias
  digitalWrite(TRIG, LOW);

  long duracion = pulseIn(ECHO, HIGH);
  long distancia = duracion * 0.034 / 2; // conversion a cm

  return distancia;
}

void setup()
{
  Serial.begin(115200);

  // pines motor 1
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // pines motor 2
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);

  // pines ultrasonido
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // estado inicial motor 1
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

void loop()
{
  long distancia = medirDistancia();
  Serial.println(distancia); // esto es para verificar si el arduino esta funcionando

  if (Serial.available())
  {
    char comando = Serial.read();

    switch (comando)
    {
    case 'S': // Adelante
      // motor 1 adelante
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 150);

      // motor 2 apagado
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 0);
      break;

    case 'B': // Retroceder (AMBOS motores prendidos porque estan en paralelo)
      // motor 1 atras
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 150);

      // motor 2 atras
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, 150);
      break;

    case 'A': // Izquierda
      // motor 1 adelante
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 120);

      // motor 2 adelante
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, 120);
      break;

    case 'D': // Derecha
      // motor 1 atras
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 120);

      // motor 2 adelante
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 120);
      break;

    case 'P': // Parar
    default:
      // motor 1 apagado
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);

      // motor 2 apagado
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 0);
      break;
    }
  }

  delay(100);
}

```
retornar al main:
[click aqui](https://github.com/Pineda47/Proyecto-de-Carrito-Remoto/blob/main/README.md)

