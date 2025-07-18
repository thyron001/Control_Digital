//=========================//
//        LIBRERÍAS        //
//=========================//
#include <Controllino.h>
#include "Stone_HMI_Define.h" // Librería oficial de HMI Stone
#include "Procesar_HMI.h"     // Librería personalizada para procesar respuestas del HMI

//=========================//
//   VARIABLES GLOBALES    //
//=========================//

// PWM del motor
const int pin_motor = CONTROLLINO_D0;       // Pin de salida PWM hacia el motor
int slider_RPM = 0;                          // Valor de referencia leído del slider del HMI
char label2_text[10];                        // Texto que se mostrará en el label2 (duty cycle)

// Conteo de pulsos y cálculo de RPM
const int entrada = CONTROLLINO_IN1;         // Pin de entrada de pulsos del sensor
volatile unsigned long conteo_pulsos = 0;    // Contador de pulsos
char label4_text[10];                        // Texto que se mostrará en el label4 (RPM)
float rpm = 0;                               // Valor actual de RPM
const uint16_t PULSOS_POR_REV = 36;          // Pulsos por revolución del encoder
const float fs = 1 / 0.05;                   // Frecuencia de muestreo (Hz)

// PID Control
float deltaU = 0;                            // Cambio en la señal de control
float u = 0;                                 // Señal de control aplicada al PWM
volatile float e[3];                         // Array de errores: e[0] actual, e[1] anterior, e[2] dos atrás
float Kp;                                    // Ganancia proporcional
float Ki;                                    // Ganancia integral
float Td;                                    // Tiempo derivativo
volatile float T = 0.05;                     // Periodo de muestreo (s)

// Control por botón físico
int estadoBoton_rojo = HIGH;                 // Estado actual del botón
int estadoAnterior_rojo = LOW;               // Estado anterior del botón
int encendido_m;                             // Estado de encendido (0 = apagado, 1 = encendido)

// Máquina de estados para el motor
typedef enum {
  APAGADO,
  ENCENDIDO
} estados_t;
estados_t estado_actual;

void iniciarEstados() {
  estado_actual = APAGADO;
}

// Temporización de actualización HMI
unsigned long t_previo = 0;
unsigned long t_previo1 = 0;

//=========================//
//     DECLARACIÓN ISR     //
//=========================//
void contarPulso(); // Prototipo de función de conteo de pulsos

//=========================//
//          SETUP          //
//=========================//
void setup() {
  Serial.begin(115200);     // Comunicación con PC
  Serial2.begin(115200);    // Comunicación con HMI

  // Inicialización de gráficas en el HMI
  STONE_push_series("line_series", "line_series1", 0);
  STONE_push_series("line_series", "line_series2", 0);
  STONE_push_series("line_series", "line_series3", 0);
  Stone_HMI_Set_Value("slider", "slider1", NULL, 0);

  // Parámetros PID iniciales
  Kp = 0.01;
  Ki = 0.015;
  Td = 0;

  // Configuración de pines
  pinMode(entrada, INPUT);
  pinMode(pin_motor, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(entrada), contarPulso, FALLING);

  // Configuración de Timer1 para generar interrupción periódica
  noInterrupts();
  TCCR1A = 0b00000000;     // Modo normal
  TCCR1B = 0b00000000;     // Modo normal
  TCCR1B |= B00000100;     // Preescaler = 256
  TIMSK1 |= B00000010;     // Habilitar interrupción por comparación
  OCR1A = 62500 / fs;      // TOP para frecuencia deseada
  interrupts();

  // Inicialización del procesamiento de HMI
  HMI_init();
  Stone_HMI_Set_Value("spin_box", "spin_box1", NULL, 1);   // Kp
  Stone_HMI_Set_Value("spin_box", "spin_box2", NULL, 15);  // Ki
  Stone_HMI_Set_Value("spin_box", "spin_box3", NULL, 0);   // Td

  iniciarEstados(); // Estado inicial
}

//=========================//
//           LOOP          //
//=========================//
void loop() {
  // Leer parámetros cada 50 ms
  if (millis() - t_previo1 >= 50) {
    if (encendido_m == 0) {
      deltaU = 0;
      Kp = 0;
      Ki = 0;
      Td = 0;
    }

    if (encendido_m == 1) {
      Kp = HMI_get_value("spin_box", "spin_box1") / 100.0;
      Ki = HMI_get_value("spin_box", "spin_box2") / 1000.0;
      Td = HMI_get_value("spin_box", "spin_box3");
    }

    slider_RPM = HMI_get_value("slider", "slider1");
    t_previo1 = millis();
  }

  // Enviar valores al HMI cada 50 ms
  if (millis() - t_previo >= 50) {
    t_previo = millis();

    dtostrf(slider_RPM, 7, 2, label2_text);  // Conversión float a texto
    dtostrf(rpm, 7, 2, label4_text);

    Stone_HMI_Set_Text("label", "label2", label2_text);
    Stone_HMI_Set_Text("label", "label4", label4_text);

    STONE_push_series("line_series", "line_series1", slider_RPM);
    STONE_push_series("line_series", "line_series2", rpm);
    STONE_push_series("line_series", "line_series3", deltaU);
  }
}

//=========================//
// INTERRUPCIÓN POR TIMER1 //
//=========================//
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0; // Reiniciar contador

  // Alternar estado de encendido con botón físico
  estadoBoton_rojo = digitalRead(CONTROLLINO_I16);
  if (estadoAnterior_rojo == HIGH && estadoBoton_rojo == LOW) {
    encendido_m = (encendido_m == 0) ? 1 : 0;
  }
  estadoAnterior_rojo = estadoBoton_rojo;

  // Calcular RPM
  rpm = (float(conteo_pulsos) * 60.0 * fs) / PULSOS_POR_REV;
  e[0] = slider_RPM - rpm;

  // Ecuación PID en forma discreta (recurrencia)
  deltaU += Kp * (e[0] - e[1]) + Ki * e[0] + Kp * (Td / T) * (e[0] - 2 * e[1] + e[2]);

  // Saturación de la señal de control
  if (deltaU > 255) deltaU = 255;
  if (deltaU < 0) deltaU = 0;

  analogWrite(pin_motor, deltaU); // Aplicar PWM

  // Enviar datos por consola
  Serial.print("\nRPM: ");
  Serial.println(rpm);
  Serial.print("Referencia: ");
  Serial.println(slider_RPM);
  Serial.print("Control: ");
  Serial.println(deltaU);
  Serial.print("Kp: ");
  Serial.println(Kp);

  // Actualizar historial de errores
  e[2] = e[1];
  e[1] = e[0];

  conteo_pulsos = 0; // Reiniciar contador de pulsos
}

//=========================//
//  INTERRUPCIÓN POR PULSO //
//=========================//
void contarPulso() {
  conteo_pulsos++; // Incrementar conteo por cada pulso
}
