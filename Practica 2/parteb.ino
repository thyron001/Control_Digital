// Práctica 2 parte b: Semaforo con maquina de estados
#include <Controllino.h>

// Definiciones para estado de salida
#define ENCENDIDO HIGH  // Nivel alto: LED encendido
#define APAGADO   LOW   // Nivel bajo: LED apagado

// Variables para temporización
unsigned long t_previo    = 0;  // Último instante registrado (ms)
unsigned long t_actual    = 0;  // Tiempo actual (ms)
int           duracion    = 0;  // Duración del estado actual (ms)

// Variables para fase de inicio
int cont_inicio     = 0;         // Contador de parpadeos inicial
int estadoAmarillo  = LOW;       // Estado actual del amarillo en INICIO

// Estructura para representar un semáforo (verde, amarillo, rojo)
struct Semaforo {
  int verde;
  int amarillo;
  int rojo;
};

// Instancias de dos semáforos con pines asignados
Semaforo Semaforo1 = { CONTROLLINO_D2, CONTROLLINO_D1, CONTROLLINO_D0 };
Semaforo Semaforo2 = { CONTROLLINO_D8, CONTROLLINO_D7, CONTROLLINO_D6 };

// Definición de estados de la máquina
typedef enum {
  INICIO,
  VERDE_1,
  AMARILLO_1,
  VERDE_2,
  AMARILLO_2
} estados_t;

estados_t estado_actual;  // Estado actual de la máquina

// -----------------------------------------------------------------------------
// setup(): Configura pines y arranca en el estado INICIO
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);  // Inicializa comunicación serial (opcional)

  // Configura pines de Semaforo1 como salida
  pinMode(Semaforo1.verde, OUTPUT);
  pinMode(Semaforo1.amarillo, OUTPUT);
  pinMode(Semaforo1.rojo, OUTPUT);

  // Configura pines de Semaforo2 como salida
  pinMode(Semaforo2.verde, OUTPUT);
  pinMode(Semaforo2.amarillo, OUTPUT);
  pinMode(Semaforo2.rojo, OUTPUT);

  // Inicia en el estado INICIO
  cambiar_estado(INICIO);
}

// -----------------------------------------------------------------------------
// loop(): Control de la máquina de estados según temporización
// -----------------------------------------------------------------------------
void loop() {
  t_actual = millis();  // Actualiza tiempo actual

  // Si ha transcurrido la duración del estado, pasamos al siguiente
  if (t_actual - t_previo >= duracion) {
    switch (estado_actual) {
      case INICIO:
        // Parpadea amarillo 5 veces antes de pasar a VERDE_1
        if (cont_inicio >= 5) {
          cambiar_estado(VERDE_1);
        } else {
          cont_inicio++;
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

// -----------------------------------------------------------------------------
// cambiar_estado(): Apaga semáforos, actualiza estado y configura LEDs/duración
// -----------------------------------------------------------------------------
void cambiar_estado(estados_t estado_nuevo) {
  apagarSemaforos();       // Apaga todos los LEDs primero
  t_previo = t_actual;     // Reinicia temporizador
  estado_actual = estado_nuevo;

  switch (estado_actual) {
    case INICIO:
      // Alterna amarillo en ambos semáforos
      estadoAmarillo = (estadoAmarillo == LOW) ? HIGH : LOW;
      digitalWrite(Semaforo1.amarillo, estadoAmarillo);
      digitalWrite(Semaforo2.amarillo, estadoAmarillo);
      duracion = 500;  // 500 ms para parpadeo inicial
      break;

    case VERDE_1:
      // Semaforo1 en verde, Semaforo2 en rojo
      digitalWrite(Semaforo1.verde, ENCENDIDO);
      digitalWrite(Semaforo2.rojo, ENCENDIDO);
      duracion = 5000;  // 5 s
      break;

    case AMARILLO_1:
      // Semaforo1 en amarillo, Semaforo2 en rojo
      digitalWrite(Semaforo1.amarillo, ENCENDIDO);
      digitalWrite(Semaforo2.rojo, ENCENDIDO);
      duracion = 5000;
      break;

    case VERDE_2:
      // Semaforo1 en rojo, Semaforo2 en verde
      digitalWrite(Semaforo1.rojo, ENCENDIDO);
      digitalWrite(Semaforo2.verde, ENCENDIDO);
      duracion = 5000;
      break;

    case AMARILLO_2:
      // Semaforo1 en rojo, Semaforo2 en amarillo
      digitalWrite(Semaforo1.rojo, ENCENDIDO);
      digitalWrite(Semaforo2.amarillo, ENCENDIDO);
      duracion = 5000;
      break;

    default:
      // Sin acción para estados no definidos
      break;
  }
}

// -----------------------------------------------------------------------------
// apagarSemaforos(): Apaga todos los LEDs de ambos semáforos
// -----------------------------------------------------------------------------
void apagarSemaforos() {
  // Semaforo1
  digitalWrite(Semaforo1.verde,    APAGADO);
  digitalWrite(Semaforo1.amarillo, APAGADO);
  digitalWrite(Semaforo1.rojo,     APAGADO);

  // Semaforo2
  digitalWrite(Semaforo2.verde,    APAGADO);
  digitalWrite(Semaforo2.amarillo, APAGADO);
  digitalWrite(Semaforo2.rojo,     APAGADO);
}
