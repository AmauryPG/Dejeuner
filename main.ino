#include "fonctionMouvement.h"
/*
https://tutorial.cytron.io/2016/10/13/make-line-following-robot-faster/
https://create.arduino.cc/projecthub/azoreanduino/simple-bluetooth-lamp-controller-using-android-and-arduino-aa2253
*/
#include <AutoPID.h>
#include <PID_AutoTune_v0/PID_AutoTune_v0.h>

//pid settings and gains
#define OUTPUT_MIN -1
#define OUTPUT_MAX 1
#define KP 0.1
#define KI 0.0
#define KD 0.0

double input, setPoint, outputValGauche, outputValDroit;

//input/output variables passed by reference, so they are updated automatically
AutoPID GauchePID(&input, &setPoint, &outputValGauche, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID DroitPID(&input, &setPoint, &outputValDroit, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

const int pinDroit = 1;
const int pinMilieu = 2;
const int pinGauche = 3;

/*
void setup()
{
  Serial.begin(9600);       
  BoardInit();

  //if input is more than 1 below or above setpoint, OUTPUT will be set to min or max respectively
  GauchePID.setBangBang(1);
  DroitPID.setBangBang(1);
  //set PID update interval to 4000ms
  GauchePID.setTimeStep(0); 
  DroitPID.setTimeStep(0); 

  pinMode(pinDroit,  OUTPUT);
  pinMode(pinMilieu, OUTPUT);
  pinMode(pinGauche, OUTPUT); 
 
  setPoint = 0;

}

void loop()
{
  input = readCapteur();

  GauchePID.run();
  DroitPID.run();

  MOTOR_SetSpeed(gauche, outputValGauche);
  MOTOR_SetSpeed(droit, -outputValDroit);

}

int readCapteur()
{
  
  if(!digitalRead(pinDroit) && digitalRead(pinMilieu) && !digitalRead(pinGauche))
  {
    //va tout droit
    return 0;
  }
  else if(!digitalRead(pinDroit) && !digitalRead(pinMilieu) && digitalRead(pinGauche))
  {
    //tourne a gauche
    return 1;
  }
  else if(digitalRead(pinDroit) && !digitalRead(pinMilieu) && !digitalRead(pinGauche))
  {
    //tourne a droit
    return -1;
  }
}
*/
/*
#include <SoftwareSerial.h> 
#define RELAY 10 
#define LIGHT 13 
SoftwareSerial btm(2,3); // rx tx 
int index = 0; 
char data[10]; 
char c; 
boolean flag = false;
void setup() { 
 pinMode(RELAY,OUTPUT); 
 pinMode(LIGHT,OUTPUT); 
 digitalWrite(RELAY,HIGH); 
 digitalWrite(LIGHT,LOW); 
 btm.begin(9600); 
} 
void loop() { 
   if(btm.available() > 0){ 
     while(btm.available() > 0){ 
          c = btm.read(); 
          delay(10); //Delay required 
          data[index] = c; 
          index++; 
     } 
     data[index] = '\0'; 
     flag = true;   
   }  
   if(flag){ 
     processCommand(); 
     flag = false; 
     index = 0; 
     data[0] = '\0'; 
   } 
} 
void processCommand(){ 
 char command = data[0]; 
 char inst = data[1]; 
 switch(command){ 
   case 'R': 
         if(inst == 'Y'){ 
           digitalWrite(RELAY,LOW); 
           btm.println("Relay: ON"); 
         } 
         else if(inst == 'N'){ 
           digitalWrite(RELAY,HIGH); 
           btm.println("Relay: OFF"); 
         } 
   break; 
   case 'L': 
         if(inst == 'Y'){ 
           digitalWrite(LIGHT,HIGH); 
           btm.println("Light: ON"); 
         } 
         else if(inst == 'N'){ 
           digitalWrite(LIGHT,LOW); 
           btm.println("Light: OFF"); 
         } 
   break; 
 } 
} */

#include <QTRSensors.h>

#define Kp 0.05            // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 2               // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
#define rightMaxSpeed 1  // max speed of the robot
#define leftMaxSpeed 1   // max speed of the robot
#define rightBaseSpeed 0.8 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 0.8  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define TIMEOUT 2500       // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN 2      // emitter is controlled by digital pin 2

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup()
{

  /* comment this part out for automatic calibration
  if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
     turn_right(); 
   else
     turn_left(); */
  qtr.calibrate(QTRReadMode::On);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(EMITTER_PIN);
  delay(20);

  delay(10000); // wait for 2s to position the bot before entering the main loop

  /* comment out for serial printing
   
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    */
}

int lastError = 0;

void loop()
{

  //retourne 1 s'il trouve la couleur noir
  int position = qtr.readLineBlack(sensorValues);
  int error = position - 2500;

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  if (rightMotorSpeed > rightMaxSpeed)
    rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed)
    leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)
    rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0)
    leftMotorSpeed = 0; // keep the motor speed positive

  {
    // move forward with appropriate speeds
    MOTOR_SetSpeed(gauche, leftMotorSpeed);
    MOTOR_SetSpeed(droit, rightMotorSpeed);
  }
}