#include <Arduino.h>
#include <MQUnifiedsensor.h>
#define trigPin 11               // ultrasonic trigger
#define echoPin 10               // ultrasonic echo
#define in1 9                    // motor pin
#define in2 8                    // motor pin
#define in3 4                    // motor pin
#define in4 3                    // motor pin
#define buzzer 7                 // buzzer pin
#define Board ("Arduino UNO")    // Board Type for MQ3
#define MQ3Pin A3                // MQ3 Pin
#define Type ("MQ-3")            // MQ Type, we're using unified MQ 
#define Voltage_Resolution (5)   // Voltage Resolution MQ3
#define ADC_Bit_Resolution (10)  // ADC_BIt Resolution MQ3
#define RatioMQ3CleanAir (60)    // RS/R0 = 60 ppm MQ3 preset value

// kalman variable
float kalmanR;
float Xt, Xt_update, Xt_prev; 
float Pt, Pt_update, Pt_prev;
float Kg, R, Q;

/// kalman filter to filter distance and ppm input
float kalman_filter(float data){
  Xt_update = Xt_prev;
  Pt_update = Pt_prev + Q;
  Kg = Pt_update / (Pt_update + R);
  Xt = Xt_update + (Kg * (data - Xt_update));
  Pt = (1 - Kg) * Pt_update;

  Xt_prev = Xt;
  Pt_prev = Pt; 

  return Xt;
}

MQUnifiedsensor MQ3(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ3Pin, Type); ///MQ3

void setup() 
{
  Serial.begin(9600);
  
  /// pinmode setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode (in1, OUTPUT);
  pinMode (in2, OUTPUT);
  pinMode (in3, OUTPUT);
  pinMode (in4, OUTPUT);
  pinMode (buzzer, OUTPUT);

  /// Kalman setup
  Pt_prev = 1;
  
  /// MQ3 Setup
  MQ3.setRegressionMethod(1); //obtaining PPM =  a*ratio^b
  MQ3.setA(0.3934); MQ3.setB(-1.504); ///Alcohol| 0.3934 | -1.504
  
  ///----------/// MQ3 CALIBRATION (ADVANCED) - DO THIS ROUTINE ONLY ON A CONTROLLED ENVIRONMENT (LAB CONDITIONS) ///----------///

  /*Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ3.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ3.calibrate(RatioMQ3CleanAir);
    Serial.print(".");
  }
  MQ3.setR0(calcR0/10);
  Serial.println("  done!.");
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);*/

  ///----------/// MQ3 CALIBRATION (ADVANCED) - DO THIS ROUTINE ONLY ON A CONTROLLED ENVIRONMENT (LAB CONDITIONS) ///----------///
  
  MQ3.serialDebug(true);
}
long duration, distance;

void loop()
{     
  
  MQ3.update(); // Update data, arduino reads the voltage from the analog pin
  float PPM = MQ3.readSensor(); // Sensor reads PPM concentration using the model, a and b values set previously or from the setup
  //float filtered_PPM = kalman_filter(PPM); /// filters the ppm reading - use this variable to instead access kalman filter

  if (PPM < 200) // main program runs when PPM < 400, ppm can be set according to needs
  {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);  
    duration = pulseIn(echoPin, HIGH);
    distance = duration/58.2;
    float filtered_distance = kalman_filter(distance);
    if(filtered_distance<30) /// distance to object can be set according to needs
      {
        digitalWrite(in1, HIGH); 
        digitalWrite(in2, LOW); 
        digitalWrite(in3, LOW); 
        digitalWrite(in4, HIGH);
        delay(1200);
      }
    else
      {
        digitalWrite(in1, HIGH); 
        digitalWrite(in2, LOW); 
        digitalWrite(in3, HIGH); 
        digitalWrite(in4, LOW);
      }  
    delay(0);
  }
  else //interrupts the main program by sounding a buzzer
  {
    digitalWrite(buzzer, HIGH); // Sound the buzzer
    delay(250);
    digitalWrite(buzzer, LOW); // Turn off the buzzer
    delay(250);
  }
}