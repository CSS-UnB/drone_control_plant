//Programa: Controle de motor brushless EMAX CF2822
//Autor : Arduino e Cia

#include <Servo.h>

Servo ServoMotor;

//Pino de controle do motor
int pino_motor = 9;
int valor;

void setup()
{
  Serial.begin(9600);
  ServoMotor.attach(pino_motor);
  //Serial.println("Aguardando 5 segundos....");
  ServoMotor.write(1000); // não lembro o por que
  delay(5000);
}

void loop()
{
  valor = 120; // 0 a 179
  int inc = 5; 
  for(; valor <= 140; valor += inc)
  {
    Serial.println(valor);
    //Envia o valor para o motor
    ServoMotor.write(valor);
    delay(2000);
  }
   
}
