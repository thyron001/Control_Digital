//LIBRERIAS
#include <Controllino.h>
#include "Stone_HMI_Define.h" // Librería ofiical de HMI Stone
#include "Procesar_HMI.h"     // Librería implementada para procesar respuestas del HMI

// VARIABLES PARA PWM DEL MOTOR
const int pin_motor        = CONTROLLINO_D0;              // Pin de salida PWM al motor
int   slider_RPM = 0;                           // Valor leído del slider del HMI
char label2_text[10];                                     // Char para mostrar el Duty Cycle en el label2 del HMI

// VARIABLES PARA CONTEO DE PULSOS y RPM
const int entrada                    = CONTROLLINO_IN1;   // Pin de entrada de pulsos
volatile unsigned long conteo_pulsos = 0;                 // Contador de pulsos
char label4_text[10];                                     // Char para mostrar las RPM en el label4 del HMI
float rpm                            = 0;                 // RPM calculadas
const uint16_t PULSOS_POR_REV        = 36;                // Número de pulsos por revolución (Datos del EPC)
const float fs                = 1/0.05; 

float deltaU;        // Cambio en la señal de control
float u=0;             // Señal de control para PWM
volatile float e[3];          // Array para almacenar los valores de error del controlador PID

float Kp;
float Ki;
float Td;

//VARIABLES PARA LOS BOTONES
int estadoBoton_rojo = HIGH;
int estadoAnterior_rojo = LOW;
int encendido_m;
//ESTADOS PARA LOS BOTONES
typedef enum{
	APAGADO,
  ENCENDIDO
}estados_t;
estados_t estado_actual;

void iniciarEstados(){
	estado_actual = APAGADO;
}

volatile float T = 0.05;

// VARIABLES PARA CONTROLAR EL TIEMPO DE ENVIO DE DATOS AL HMI
unsigned long t_previo=0;
unsigned long t_previo1=0;

// FUNCIONES ADICIONALES 
void contarPulso();

void setup() {
  Serial.begin(115200);   // Comunicación serial con el PC
  Serial2.begin(115200);  // Comunicación serial con el HMI
  STONE_push_series("line_series", "line_series1", 0); //Envía un valor del eje X a graficar en el line_series1 que se pondrá al final
  STONE_push_series("line_series", "line_series2", 0); //Envía un valor del eje X a graficar en el line_series2 que se pondrá al final
  STONE_push_series("line_series", "line_series3", 0); //Envía un valor del eje X a graficar en el line_series2 que se pondrá al final
  Stone_HMI_Set_Value("slider", "slider1", NULL, 0);
  Kp = 0.01;
  Ki = 0.015;
  Td = 0;
  
  pinMode(entrada, INPUT);
  pinMode(pin_motor, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(entrada), contarPulso, FALLING);
  noInterrupts();
  TCCR1A = 0b00000000;     // Todo apagado, modo normal registro A. Counter1 del ATMEGA2560
  TCCR1B = 0b00000000;     // Todo apagado, modo normal registro B. Coun ter1 del ATMEGA2560
  TCCR1B |= B00000100;     // Configuración de preescaler a 256 (BIT CS12)
  TIMSK1 |= B00000010;     // Habilitar interrupción por comparación usando el registro TIMSK1 (modo CTC)
  OCR1A = 62500/fs; // Establecer valor TOP para generar interrupción cada 1 segundo (valor a comparar calculado)
  interrupts();
  HMI_init(); // Inicialización del sistema de colas para procesar las respuestas del HMI
  Stone_HMI_Set_Value("spin_box", "spin_box1", NULL, 1);
  Stone_HMI_Set_Value("spin_box", "spin_box2", NULL, 15);
  Stone_HMI_Set_Value("spin_box", "spin_box3", NULL, 0);

  iniciarEstados();
}

void loop() {
  if(millis()-t_previo1>=50){  

  if (encendido_m == 0){
    deltaU = 0;
    Kp=0;Ki=0;Td=0;
  }

  if (encendido_m == 1){
    Kp = HMI_get_value("spin_box", "spin_box1") / 100;
    Ki = HMI_get_value("spin_box", "spin_box2") / 1000;
    Td = HMI_get_value("spin_box", "spin_box3");
  }

    slider_RPM= HMI_get_value("slider", "slider1");  // Solicita el valor del widget slider1    
    t_previo1=millis();
  }
  
  if(millis()-t_previo>=50){
    t_previo=millis();
    dtostrf(slider_RPM, 7, 2, label2_text);     // Convertir float a char
    dtostrf(rpm, 7, 2, label4_text);                  // Convertir float a char
    Stone_HMI_Set_Text("label","label2",label2_text); // Envía el texto del duty cycle al label2
    Stone_HMI_Set_Text("label","label4",label4_text); // Envía el texto de las rpm al label4

    //Envía un valor del eje X a graficar en el line_series1 que se pondrá al final
    STONE_push_series("line_series", "line_series1", slider_RPM);\ 
    //Envía un valor del eje X a graficar en el line_series2 que se pondrá al final
    STONE_push_series("line_series", "line_series2", rpm); 
    STONE_push_series("line_series", "line_series3", deltaU); 
    
  }
}


// Interrupción por TIMER1 para muestrear las RPM debido a que la librería del HMI se demora mucho
ISR(TIMER1_COMPA_vect){     
  TCNT1=0;          // Resetea el timer (Timer 1 a cero)

  //*****************
  //            EMPLEO DEL ESTADO DE ENCENDIDO
  //*****************

  estadoBoton_rojo = digitalRead(CONTROLLINO_I16);
    if (estadoAnterior_rojo == HIGH && estadoBoton_rojo == LOW) {
      encendido_m = (encendido_m == 0) ? 1 : 0;  // Alternar estado de encendido
    }

    estadoAnterior_rojo = estadoBoton_rojo;

    Serial.println(encendido_m);


  // Calcular RPM (pulsos por segundo / pulsos por revolución) * 60
  rpm = (float(conteo_pulsos)*60)*fs / (PULSOS_POR_REV);
  e[0]= slider_RPM - rpm;

  deltaU += Kp * (e[0] - e[1]) + Ki * e[0] + Kp * (Td/T) * (e[0] - 2*e[1] + e[2]);

  if (deltaU > 255)
      deltaU = 255;
    if (deltaU < 0)
      deltaU = 0;

  analogWrite(pin_motor, deltaU);

  // Mostrar las RPM
  Serial.print("\nRPM: ");
  Serial.println(rpm);
  Serial.print("Referencia: ");
  Serial.println(slider_RPM);
  Serial.print("Control: ");
  Serial.println(deltaU);
  Serial.print("Kp: ");
  Serial.println(Kp);

  conteo_pulsos=0;  // Resetea los pulsos 
}

// Interrupción por Hardware para contar los pulsos del motor
void contarPulso() {
  conteo_pulsos++;  // Incrementar contador al detectar pulso
}
