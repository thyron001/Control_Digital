#include <Controllino.h>

#define ENCENDIDO HIGH
#define APAGADO   LOW

unsigned long t_previo = 0;
int intervalo;
unsigned long t_actual = 0;
int cont_inicio = 0;
int duracion;
int estadoAmarillo = LOW;

//Creacion de estructura Semaforo
struct Semaforo {
  int verde;
  int amarillo;
  int rojo;
};

// Creacion de semaforos mediante estructura semaforo
Semaforo Semaforo1 = {CONTROLLINO_D2, CONTROLLINO_D1, CONTROLLINO_D0};
Semaforo Semaforo2 = {CONTROLLINO_D8, CONTROLLINO_D7, CONTROLLINO_D6};

//Definicion de estados
typedef enum{
	  INICIO,
  	VERDE_1,
  	AMARILLO_1,
  	VERDE_2,
  	AMARILLO_2,
}estados_t;

estados_t estado_actual;


void setup()
{
  Serial.begin(9600);

  pinMode(Semaforo1.verde, OUTPUT);
  pinMode(Semaforo1.amarillo, OUTPUT);
  pinMode(Semaforo1.rojo, OUTPUT);

  pinMode(Semaforo2.verde, OUTPUT);
  pinMode(Semaforo2.amarillo, OUTPUT);
  pinMode(Semaforo2.rojo, OUTPUT);

  cambiar_estado(INICIO);
}

void loop()
{
  t_actual = millis();

  if (t_actual - t_previo >= duracion) {

    switch (estado_actual) {
      case INICIO:
        if (cont_inicio == 5 ) {
          cambiar_estado(VERDE_1);       
        }else{
          cont_inicio ++;
          cambiar_estado(INICIO);
        }
        break;
      case VERDE_1:
        cambiar_estado(AMARILLO_1);
        break;
      case AMARILLO_1:
        cambiar_estado(VERDE_2);
        break;
      case VERDE_2:
        cambiar_estado(AMARILLO_2);
        break;
      case AMARILLO_2:
        cambiar_estado(VERDE_1);
        break;
    }
  }
}

void cambiar_estado(estados_t estado_nuevo) {

  apagarSemaforos();

  t_previo = t_actual;

  estado_actual = estado_nuevo;  

  switch(estado_actual){
  case INICIO:
    estadoAmarillo = (estadoAmarillo == LOW) ? HIGH : LOW;
    digitalWrite(Semaforo1.amarillo, estadoAmarillo);
    digitalWrite(Semaforo2.amarillo, estadoAmarillo);
    duracion = 500; 
  	break;
  case VERDE_1:
    digitalWrite(Semaforo1.verde, ENCENDIDO);
    digitalWrite(Semaforo2.rojo, ENCENDIDO);
    duracion = 5000;
    break;
  case AMARILLO_1:
    digitalWrite(Semaforo1.amarillo, ENCENDIDO);
    digitalWrite(Semaforo2.rojo, ENCENDIDO);
    duracion = 5000;
    break;
  case VERDE_2:
    digitalWrite(Semaforo1.rojo, ENCENDIDO);
    digitalWrite(Semaforo2.verde, ENCENDIDO);
    duracion = 5000;
    break;
  case AMARILLO_2:
    digitalWrite(Semaforo2.amarillo, ENCENDIDO);
    digitalWrite(Semaforo1.rojo, ENCENDIDO);
    duracion = 5000;
    break;
  default:
    break;
  }
}

void apagarSemaforos() {
  digitalWrite(Semaforo1.verde,     APAGADO);
  digitalWrite(Semaforo1.amarillo,  APAGADO);
  digitalWrite(Semaforo1.rojo,      APAGADO);
  digitalWrite(Semaforo2.verde,     APAGADO);
  digitalWrite(Semaforo2.amarillo,  APAGADO);
  digitalWrite(Semaforo2.rojo,      APAGADO);
}

