// Práctica 2 parte a: Encendido y apagado de LEDs mediante dos botones
#include <Controllino.h>  // Librería para hardware Controlino

// -----------------------------------------------------------------------------
// Constantes y variables globales
// -----------------------------------------------------------------------------
const unsigned long INTERVALO = 500;  // Intervalo en ms para el cambio de LED
unsigned long t_previo      = 0;      // Último instante registrado
unsigned long t_actual      = 0;      // Tiempo actual desde millis()

int i    = 0;    // Índice del LED activo
int flag = 0;    // Dirección de secuencia:  1 = adelante, -1 = atrás, 0 = detenido

// Arreglo con pines de los 9 LEDs y puntero para recorrerlos
int LEDS[9] = {
  CONTROLLINO_D0, CONTROLLINO_D6, CONTROLLINO_D12,
  CONTROLLINO_D13, CONTROLLINO_D14, CONTROLLINO_D8,
  CONTROLLINO_D2, CONTROLLINO_D1,  CONTROLLINO_D7
};
int* led = LEDS;  // Puntero al primer elemento de 'LEDS'

// -----------------------------------------------------------------------------
// Función de setup: configura pines y comunicación serial
// -----------------------------------------------------------------------------
void setup() {
  // Configurar cada pin de LED como salida
  for (int j = 0; j < 9; j++) {
    pinMode( *(led + j), OUTPUT );
  }

  // Configurar botones como entradas
  pinMode(CONTROLLINO_I16, INPUT);  // Botón: inicia secuencia hacia adelante
  pinMode(CONTROLLINO_I17, INPUT);  // Botón: inicia secuencia hacia atrás
  pinMode(CONTROLLINO_I18, INPUT);  // Botón: reset de secuencia

  Serial.begin(9600);  // Inicializa comunicación serial (opcional)
}

// -----------------------------------------------------------------------------
// Función para apagar todos los LEDs del tablero
// -----------------------------------------------------------------------------
void apaga_todo() {
  for (int j = 0; j < 9; j++) {
    digitalWrite(*(led + j), LOW);
  }
}

// -----------------------------------------------------------------------------
// Bucle principal: lectura de botones y control de secuencia de LEDs
// -----------------------------------------------------------------------------
void loop() {
  // --- Lectura de botones ---
  if (digitalRead(CONTROLLINO_I16) == HIGH) {
    // Botón 16 presionado: secuencia hacia adelante
    apaga_todo();
    i       = -1;   // Se iniciará en LED 0 tras incrementar
    flag    =  1;   // Dirección positiva
    t_previo = 0;   // Reinicia temporizador
  }

  if (digitalRead(CONTROLLINO_I17) == HIGH) {
    // Botón 17 presionado: secuencia hacia atrás
    apaga_todo();
    i       =  9;   // Se iniciará en LED 8 tras decrementar
    flag    = -1;   // Dirección negativa
    t_previo = 0;
  }

  if (digitalRead(CONTROLLINO_I18) == HIGH) {
    // Botón 18 presionado: detiene y resetea secuencia
    apaga_todo();
    flag    =  0;   // Secuencia detenida
    t_previo = 0;
  }

  delay(10);  // Pequeño retardo para evitar rebotes de los botones

  // --- Control de temporización no bloqueante ---
  t_actual = millis();
  if (flag != 0 && (t_actual - t_previo >= INTERVALO)) {
    // Apaga el LED actual
    digitalWrite(*(led + i), LOW);

    // Actualiza el tiempo de referencia
    t_previo = t_actual;

    // Avanza o retrocede el índice según 'flag'
    i += flag;

    // Ajuste circular del índice
    if (i < 0 && flag == -1) {
      i = 8;    // Salta al último LED
    }
    else if (i > 8 && flag == 1) {
      i = 0;    // Salta al primer LED
    }

    // Enciende el siguiente LED
    digitalWrite(*(led + i), HIGH);
  }
}
