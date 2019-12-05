#include "fonctionMouvement.h"

/*
https://tutorial.cytron.io/2016/10/13/make-line-following-robot-faster/
https://create.arduino.cc/projecthub/azoreanduino/simple-bluetooth-lamp-controller-using-android-and-arduino-aa2253
*/

#define rightBaseSpeed 0.2 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 0.2  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define EMITTER_PIN 9      // emitter is controlled by digital pin 2

/*********************************************************************************************************************************
 *                                                          General                                                              * 
*********************************************************************************************************************************/

void avancer(float distance, float vitesse)
{
  ENCODER_Reset(gauche);
  ENCODER_Reset(droit);

  while (ENCODER_Read(gauche) <= getDistanceEncodeur(distance))
  {
    MOTOR_SetSpeed(gauche, vitesse);
    MOTOR_SetSpeed(droit, vitesse);
  }

  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droit, 0);
}

void reculer(float distance, float vitesse)
{
  ENCODER_Reset(gauche);
  ENCODER_Reset(droit);

  while (ENCODER_Read(gauche) >= -getDistanceEncodeur(distance))
  {
    MOTOR_SetSpeed(gauche, -vitesse);
    MOTOR_SetSpeed(droit, -vitesse);
  }

  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droit, 0);
}

void tournerGauche(float vitesse, int offset, float angle)
{
  ENCODER_Reset(gauche);
  ENCODER_Reset(droit);

  while (-getAngleEncodeur(angle) + offset <= ENCODER_Read(gauche))
  {
    MOTOR_SetSpeed(gauche, -0.2);
    MOTOR_SetSpeed(droit, 0.2);
  }

  ENCODER_Reset(gauche);
  ENCODER_Reset(droit);
  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droit, 0);
}

void tournerDroit(float vitesse, int offset, float angle)
{
  ENCODER_Reset(gauche);
  ENCODER_Reset(droit);

  while (getAngleEncodeur(angle) + offset >= ENCODER_Read(gauche))
  {
    MOTOR_SetSpeed(gauche, 0.2);
    MOTOR_SetSpeed(droit, -0.2);
  }

  ENCODER_Reset(gauche);
  ENCODER_Reset(droit);
  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droit, 0);
}

void tournerSuiveurLigneGaucher()
{
  while (getPosition() != 32)
  {
    //Serial.println(getPosition());
    MOTOR_SetSpeed(gauche, -0.1);
    MOTOR_SetSpeed(droit, 0.1);
  }
  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droit, 0);
}

void tournerSuiveurLigneDroit()
{
  while (getPosition() != 16)
  {
    Serial.println(getPosition());
    MOTOR_SetSpeed(gauche, 0.1);
    MOTOR_SetSpeed(droit, -0.1);
  }
  delay(800);
  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droit, 0);
}

/*********************************************************************************************************************************
 *                                                          PID                                                                 *
 *                                                     *Pas toucher*                                                            *
*********************************************************************************************************************************/

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
int seuil = 500;
int lastError = 0;

void SuiveurLigneInit()
{

  /* comment this part out for automatic calibration
  if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
     turn_right(); 
   else
     turn_left(); */
  qtr.calibrate(QTRReadMode::On);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A6, A7, A10, A11, A12, A13}, SensorCount);
  qtr.setEmitterPin(EMITTER_PIN);
}

void printSuiveurLigne()
{
  qtr.read(sensorValues);

  for (int i = 0; i < SensorCount; i++)
  {
    
    Serial.print(sensorValues[i] );
    /*
    if(i == 0 || i == 1)
    {
    Serial.print(sensorValues[i] );

    }else{
    Serial.print(sensorValues[i] );

    }*/
    Serial.print("\t");
  }
  Serial.println();
  delay(100);
}

///////////////////////////////4 centre////////////////////////

int getPosition()
{
  qtr.read(sensorValues);
  return (sensorValues[0] > 720) + 2 * (sensorValues[1] > 620) + 4 * (sensorValues[2] > 580) + 8 * (sensorValues[3] > 520) + 16 * (sensorValues[4] > 520) + 32 * (sensorValues[5] > 550);
}

int SuiveurLigne()
{
  int position = getPosition();

  float motorSpeedGauche = 0;
  float motorSpeedDroit = 0;

  switch (position)
  {
  case 1:
    motorSpeedDroit = 0.25;
    motorSpeedGauche = 0;
    break;
  case 2:
    motorSpeedDroit = 0.1;
    motorSpeedGauche = 0;
    break;
  case 4:
    motorSpeedDroit = 0;
    motorSpeedGauche = 0;
    break;
  case 8:
    motorSpeedDroit = 0;
    motorSpeedGauche = 0.1;
    break;
  case 16:
    motorSpeedDroit = 0;
    motorSpeedGauche = 0.2;
    break;
  case 32:
    motorSpeedDroit = 0;
    motorSpeedGauche = 0.25;
    break;
  }

  float rightMotorSpeed = rightBaseSpeed + motorSpeedDroit;
  float leftMotorSpeed = leftBaseSpeed + motorSpeedGauche;

  if (rightMotorSpeed > OUTPUT_MAX)
    rightMotorSpeed = OUTPUT_MAX; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > OUTPUT_MAX)
    leftMotorSpeed = OUTPUT_MAX; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0.2)
    rightMotorSpeed = 0.2; // keep the motor speed positive
  if (leftMotorSpeed < 0.2)
    leftMotorSpeed = 0.2; // keep the motor speed positive

  if (position == 63)
  {
    MOTOR_SetSpeed(gauche, 0);
    MOTOR_SetSpeed(droit, 0);
    return 1;
  }

  // move forward with appropriate speeds
  MOTOR_SetSpeed(gauche, leftMotorSpeed);
  MOTOR_SetSpeed(droit, rightMotorSpeed);
  return 0;
}

void SuiveurLigneArret()
{
  while (SuiveurLigne() == 0)
    ;

  ENCODER_Reset(gauche);
  ENCODER_Reset(droit);
  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droit, 0);
  delay(200);
}

/*********************************************************************************************************************************
 *                                                          Pelle                                                                * 
*********************************************************************************************************************************/

void prendreToast()
{
  monterPelle(140); 
  cuitLaToast();
}

void deposeToast()
{
  descendrePelle(20);
  delay(2000);
  cuitLaToast();
}
void monterPelle(int angle)
{
  SERVO_SetAngle(0, angle);
}

void descendrePelle(int angle)
{
  SERVO_SetAngle(0, angle);
}

void allume_DEL()
{
  digitalWrite(output_pin, HIGH);
}

void ferme_DEL()
{
  digitalWrite(output_pin, LOW);
}

void cuitLaToast()
{
  for (int i = 0; i < 500; i++)
  {
    Serial.println(analogRead(test_pin));
    if (analogRead(test_pin) < sensitivity)
    {
      allume_DEL();
    }
    else
    {
      ferme_DEL();
    }
    delay(10);
  }
}


/*********************************************************************************************************************************
 *                                                         Aligneur                                                              * 
*********************************************************************************************************************************/

struct scanner
{
  int distance;
  int scan_number;
};

int values_sensor0[MAXVALUES];
int values_sensor1[MAXVALUES];
int values_sensor2[MAXVALUES];

struct scanner scan_array1[MAXSCAN];
struct scanner scan_array2[MAXSCAN];

void scan(int pulseGauche, int pulseDroit, int sens, int sensor, int sensor_table[],
          struct scanner scan_array[], int length, int position)
{
  ENCODER_ReadReset(gauche);
  ENCODER_ReadReset(droit);
  if (sens == 1)
  {
    while (ENCODER_Read(gauche) <= pulseGauche)
    {
      MOTOR_SetSpeed(gauche, 0.2);
    }
    MOTOR_SetSpeed(gauche, 0);
    while (abs(ENCODER_Read(droit)) <= pulseDroit)
    {
      MOTOR_SetSpeed(droit, -0.2);
    }
    MOTOR_SetSpeed(droit, 0);
    delay(100);
    scan_array[position].distance = check_distance(35, sensor, sensor_table, length);
    Serial.println(scan_array[position].distance);
    scan_array[position].scan_number = position;
  }
  else if (sens == 0)
  {
    while (abs(ENCODER_Read(gauche)) <= pulseGauche)
    {
      MOTOR_SetSpeed(gauche, -0.2);
    }
    MOTOR_SetSpeed(gauche, 0);
    while (ENCODER_Read(droit) <= pulseDroit)
    {
      MOTOR_SetSpeed(droit, 0.2);
    }
    MOTOR_SetSpeed(droit, 0);
    delay(100);
    scan_array[position].distance = check_distance(35, sensor, sensor_table, length);
    Serial.println(scan_array[position].distance);
    scan_array[position].scan_number = position;
  }
  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droit, 0);
}

void turn(int pulseGauche, int pulseDroit, int sens)
{
  ENCODER_ReadReset(gauche);
  ENCODER_ReadReset(droit);
  if (sens == 1)
  {
    while (ENCODER_Read(gauche) <= pulseGauche)
    {
      MOTOR_SetSpeed(gauche, 0.2);
    }
    MOTOR_SetSpeed(gauche, 0);
    while (abs(ENCODER_Read(droit)) <= pulseDroit)
    {
      MOTOR_SetSpeed(droit, -0.2);
    }
    MOTOR_SetSpeed(droit, 0);
  }
  else if (sens == 0)
  {
    while (abs(ENCODER_Read(gauche)) <= pulseGauche)
    {
      MOTOR_SetSpeed(gauche, -0.2);
    }
    MOTOR_SetSpeed(gauche, 0);
    while (ENCODER_Read(gauche) <= pulseDroit)
    {
      MOTOR_SetSpeed(droit, 0.2);
    }
    MOTOR_SetSpeed(droit, 0);
  }
  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droit, 0);
}

int find_smallest_distance(struct scanner array[])
{
  int i, value;
  for (i = 0; i < MAXSCAN; i++)
  {
    if (i == 0)
    {
      value = array[i].scan_number;
    }
    if (array[i].distance < array[value].distance)
    {
      value = array[i].scan_number;
    }
  }
  return value;
}

int findSmallestDistance(struct scanner array[])
{
  int small = array[0].distance;
  int index = 0;

  for (int i = 1; i < MAXSCAN; i++)
  {
    if (array[i].distance < small)
    {
      small = array[i].distance;
      index = i;
    }
  }
  return index;
}

int find_glass(struct scanner scan_array1[], struct scanner scan_array2[], int sensor_table1[], int sensor1, int sensor2,
               int sensor_table2[], int length_sensor_table, int num_scans)
{
  int smallest_value_sensor1, smallest_value_sensor2;
  Serial.println("Looking for a glass");
  for (int i = 0; i < num_scans; i++)
  {
    scan(50, 50, 1, sensor1, sensor_table1, scan_array1, length_sensor_table, i);
    delay(50);
  }
  Serial.println("Changing scanner");
  delay(1000);
  for (int i = 0; i < num_scans; i++)
  {
    scan(50, 50, 0, sensor2, sensor_table2, scan_array2, length_sensor_table, i);
    delay(50);
  }
  smallest_value_sensor1 = find_smallest_distance(scan_array1);
  smallest_value_sensor2 = find_smallest_distance(scan_array2);
  Serial.println("-------");
  Serial.println(scan_array1[num_scans - smallest_value_sensor2].distance);
  Serial.println(smallest_value_sensor1);
  Serial.println("--------");

  if (scan_array2[smallest_value_sensor2].distance != 100)
  {
    if (abs(scan_array1[smallest_value_sensor1].distance - scan_array2[smallest_value_sensor2].distance) <= 5)
    {
      Serial.println("Found a glass");

      int direction = (MAXSCAN - smallest_value_sensor2) - 5;
      if (direction < 0)
        direction = 0;
      for (int i = 0; i < direction; i++)
      {
        turn(50, 50, 1);
        delay(150);
      }
      avancer(scan_array2[smallest_value_sensor2].distance + 5, 0.2);
    }
    else
    {
      Serial.println("No glass found");
    }
  }
  else
  {
    Serial.println("No glass found");
  }

  return 0;
}

int find_plate(struct scanner array1[], struct scanner array2[], int sensor_table1[], int sensor1, int sensor2,
               int sensor_table2[], int length_sensor_table, int num_scans)
{
  int i, smallest_value_sensor1, smallest_value_sensor2;
  Serial.println("Looking for a plate");
  for (i = 0; i < num_scans - 5; i++)
  {
    scan(50, 50, 1, sensor1, sensor_table1, scan_array1, length_sensor_table, i);
    delay(50);
  }
  Serial.println("Changing scanner");
  delay(1000);
  for (i = 0; i < num_scans - 5; i++)
  {
    scan(50, 50, 0, sensor2, sensor_table2, scan_array2, length_sensor_table, i);
    delay(50);
  }
  smallest_value_sensor1 = find_smallest_distance(scan_array1);
  smallest_value_sensor2 = find_smallest_distance(scan_array2);
  if (smallest_value_sensor1 != 100)
  {
    //avance la distance donnee par un des capteur
    Serial.println("Plate found");
    for (i = 0; i < smallest_value_sensor1; i++)
    {
      turn(50, 50, 1);
      delay(150);
    }
  }
  else
  {
    Serial.println("No plate found");
    //tourner a gauche ou a droite et refaire le scan jusqace qu'il trouve un verre
  }
  return 0;
}
void TrouverVerre()
{
  find_glass(scan_array1, scan_array2, values_sensor0, 0, 1, values_sensor1, MAXVALUES, MAXSCAN);
}

void TrouverAssiette()
{
  find_plate(scan_array1, scan_array2, values_sensor0, 0, 1, values_sensor1, MAXVALUES, MAXSCAN);
}

/*********************************************************************************************************************************
 *                                                         Pompe                                                             * 
*********************************************************************************************************************************/

void videPompe()
{
  digitalWrite(pin_pompe, HIGH);
  delay(tempsPourVider);
  digitalWrite(pin_pompe, LOW);
}