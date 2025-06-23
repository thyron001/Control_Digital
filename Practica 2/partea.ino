// Práctica 2: Encendido y apagado de un LED del tablero mediante dos botones
#include <Controllino.h> //Librería de controllino

int i = 0;
int flag;
unsigned long t_previo = 0;
unsigned long intervalo = 500;
unsigned long t_actual = 0;
int LEDS[9] = {CONTROLLINO_D0,CONTROLLINO_D6,CONTROLLINO_D12,CONTROLLINO_D13,CONTROLLINO_D14,CONTROLLINO_D8,CONTROLLINO_D2,CONTROLLINO_D1,CONTROLLINO_D7};
int* led = LEDS;

void setup() {
  pinMode(CONTROLLINO_D0, OUTPUT);   // led como salida
  pinMode(CONTROLLINO_D6, OUTPUT);   // led como salida
  pinMode(CONTROLLINO_D12, OUTPUT);   // led como salida
  pinMode(CONTROLLINO_I16, INPUT);  // boton_encendido como entrada
  pinMode(CONTROLLINO_I17, INPUT);  // boton_encendido como entrada
  pinMode(CONTROLLINO_I18, INPUT);  // boton_encendido como entrada

  Serial.begin(9600);
}

void apaga_todo(){
  for (int j = 0; j < 9; j++) {
        digitalWrite(*(led + j), LOW);
    }
}

void loop() {
  //secuencia 1
  if (digitalRead(CONTROLLINO_I16) == HIGH) {
    apaga_todo();
    i = -1;
    flag = 1;
    t_previo = 0;
  }

  //secuencia 2
  if (digitalRead(CONTROLLINO_I17) == HIGH) {
    apaga_todo();
    i = 9;
    flag = -1;
    t_previo = 0;
  }

  //reset
  if (digitalRead(CONTROLLINO_I18) == HIGH) {
    apaga_todo();
    flag = 0;
    t_previo = 0;
  }

  delay(10); // Pequeño retardo para evitar rebotes


  t_actual = millis();

  if(t_actual - t_previo >= intervalo && flag != 0){

    digitalWrite(*(led + i), LOW);
    t_previo = t_actual;

    //Aumentar el corredor de memoria
    i = i + flag;
    
    if(i < 0 && flag == -1){
      i = 8;
    }

    //Reiniciar el corredor de memoria
    if(i > 8 && flag == 1){
      i = 0;
    }

    digitalWrite(*(led + i), HIGH);
    
  }

}






