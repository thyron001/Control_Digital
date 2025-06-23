// Practica 2: Encender todos los LEDs del tablero secuencialmente

//Libreria de la tarjeta
#include <Controllino.h>

// Variables para control del tiempo y el índice de recorrido
int i = 0;

// Tiempo en el que se realizo el ultimo cambio
unsigned long t_previo = 0;
// Intervalo entre cambios en milisegundos (medio segundo)
unsigned long intervalo = 500;
// Tiempo actual proporcionado por millis() 
unsigned long t_actual = 0;

// Arreglo con los pines digitales de los LEDs en el orden deseado de encendido
int LEDS[9] = {
CONTROLLINO_D0,
CONTROLLINO_D6,
CONTROLLINO_D12,
CONTROLLINO_D13,
CONTROLLINO_D14,
CONTROLLINO_D8,
CONTROLLINO_D2,
CONTROLLINO_D1,
CONTROLLINO_D7};

// Puntero que apunta al primer elemento del arreglo de LEDs
int* led = LEDS;

void setup() {
}

void loop() {
  // Actualizacion del tiempo actual
  t_actual = millis();

  // Enciende el primer LED si es la primera iteracion
  if(i == 0){
      digitalWrite(*(led), HIGH);
  }

  // Verifica si ha transcurrido el intervalo definido
  if(t_actual - t_previo >= intervalo){

    // Apaga el LED actual
    digitalWrite(*(led + i), LOW);  
    // Actualiza el tiempo previo
    t_previo = t_actual;            

    // Incrementa el indice para apuntar al siguiente LED
    i = i + 1;                      

    // Enciende el nuevo LED
    digitalWrite(*(led + i), HIGH); 

    // Reinicio del indice para repetir la secuencia
    if(i == 9){
      i = 0;
    }
  }
}
