#include <Controllino.h>             // Librería para controlar pines del PLC CONTROLLINO
#include "Stone_HMI_Define.h"       // Definiciones para la pantalla táctil HMI Stone
#include "Procesar_HMI.h"           // Funciones de procesamiento para comunicación con HMI

// Definición de pines de salida para los LEDs
const int led_rojo = CONTROLLINO_D0;
const int led_amarillo = CONTROLLINO_D1;

// Variables de PWM para controlar brillo
int pwmValue_rojo = 100;            
int pwmValue_amarillo = 100;        

// Ciclos de trabajo expresados en porcentaje (0-100%)
float dutyCyclePercent_rojo = 100;             
float dutyCyclePercent_amarillo = 100;         

// Banderas para indicar si los LEDs están encendidos o apagados
int led_1_encendido = 0;
int led_2_encendido = 0;

// Variables de temporización para control de actualización
unsigned long t_previo = 0;
unsigned long t_actual = 0;

// Variables para detectar flancos de bajada en los botones
int estadoBoton_rojo = HIGH;
int estadoAnterior_rojo = HIGH;
int estadoBoton_amarillo = HIGH;
int estadoAnterior_amarillo = HIGH;

void setup() {
  Serial.begin(115200);             // Comunicación serial con el PC
  Serial2.begin(115200);            // Comunicación serial con la pantalla HMI

  pinMode(led_rojo, OUTPUT);        // LED rojo como salida
  pinMode(led_amarillo, OUTPUT);    // LED amarillo como salida

  HMI_init();                       // Inicializa sistema de comunicación con la HMI

  // Inicializa los spin boxes de la HMI en cero
  Stone_HMI_Set_Value("spin_box", "spin_box1", NULL, 0);
  Stone_HMI_Set_Value("spin_box", "spin_box2", NULL, 0);
}

void loop() {
  // Lectura de botones conectados a entradas digitales
  estadoBoton_rojo = digitalRead(CONTROLLINO_I16);
  estadoBoton_amarillo = digitalRead(CONTROLLINO_I17);

  // Detección de flanco de bajada (pulsación) en botón rojo
  if (estadoAnterior_rojo == HIGH && estadoBoton_rojo == LOW) {
    led_1_encendido = (led_1_encendido == 0) ? 1 : 0;  // Alternar estado
    t_previo = 0;  // Reiniciar temporizador
  }

  // Detección de flanco de bajada (pulsación) en botón amarillo
  if (estadoAnterior_amarillo == HIGH && estadoBoton_amarillo == LOW) {
    led_2_encendido = (led_2_encendido == 0) ? 1 : 0;  // Alternar estado
    t_previo = 0;  // Reiniciar temporizador
  }

  // Guardar estado actual para comparación futura
  estadoAnterior_rojo = estadoBoton_rojo;
  estadoAnterior_amarillo = estadoBoton_amarillo;

  t_actual = millis();  // Actualizar tiempo

  // Ejecutar cada 50 milisegundos
  if (t_actual - t_previo >= 50) {
    t_previo = t_actual;

    // Obtener valores actualizados desde la HMI
    dutyCyclePercent_rojo = HMI_get_value("spin_box", "spin_box1");
    dutyCyclePercent_amarillo = HMI_get_value("spin_box", "spin_box2");

    // Control LED rojo
    if (dutyCyclePercent_rojo >= 0 && dutyCyclePercent_rojo <= 100 && led_1_encendido == 1) {
      pwmValue_rojo = map(dutyCyclePercent_rojo, 0, 100, 0, 255);  // Convertir a valor PWM
      analogWrite(led_rojo, pwmValue_rojo);  // Aplicar PWM

      Serial.print(dutyCyclePercent_rojo);
      Serial.print(" -> PWM value: ");
      Serial.println(pwmValue_rojo);
    } else {
      Serial.println("Ingresa un valor entre 0 y 100.");
      analogWrite(led_rojo, 0);  // Apagar si el valor no es válido o el LED está apagado
    }

    // Control LED amarillo
    if (dutyCyclePercent_amarillo >= 0 && dutyCyclePercent_amarillo <= 100 && led_2_encendido == 1) {
      pwmValue_amarillo = map(dutyCyclePercent_amarillo, 0, 100, 0, 255);
      analogWrite(led_amarillo, pwmValue_amarillo);

      Serial.print(dutyCyclePercent_amarillo);
      Serial.print(" -> PWM value: ");
      Serial.println(pwmValue_amarillo);
    } else {
      Serial.println("Ingresa un valor entre 0 y 100.");
      analogWrite(led_amarillo, 0);  // Apagar si el valor no es válido o el LED está apagado
    }
  }
}
