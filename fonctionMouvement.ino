#include "fonctionMouvement.h"
/*
https://tutorial.cytron.io/2016/10/13/make-line-following-robot-faster/
https://create.arduino.cc/projecthub/azoreanduino/simple-bluetooth-lamp-controller-using-android-and-arduino-aa2253
*/

#define rightBaseSpeed 0.8 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 0.8  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define EMITTER_PIN 9      // emitter is controlled by digital pin 2




/*********************************************************************************************************************************
 *                                                          Pelle                                                                * 
*********************************************************************************************************************************/

void PainInit()
{
    pinMode(capteurDroit, INPUT);
    pinMode(capteurGauche, INPUT);
}

void actionPain()
{
    if (digitalRead(capteurGauche) && digitalRead(capteurDroit))
    {
        //baisser la pelle

        //attendre le pain
        delay(1000);

        //monter la pelle
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
        MOTOR_SetSpeed(droit, 0);
        while (abs(ENCODER_Read(droit)) <= pulseDroit)
        {
            MOTOR_SetSpeed(droit, -0.2);
        }
        MOTOR_SetSpeed(droit, 0);
        delay(100);
        scan_array[position].distance = check_distance(50, sensor, sensor_table, length);
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
        scan_array[position].distance = check_distance(50, 0, sensor_table, length);
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
        while (ENCODER_Read(droit) <= pulseDroit)
        {
            MOTOR_SetSpeed(droit, 0.2);
        }
        MOTOR_SetSpeed(droit, 0);
    }
    MOTOR_SetSpeed(gauche, 0);
    MOTOR_SetSpeed(droit, 0);
}

int find_glass(struct scanner scan_array1[], struct scanner scan_array2[], int sensor_table1[], int sensor1, int sensor2,
               int sensor_table2[], int length_sensor_table, int num_scans)
{
    int i, smallest_value_sensor1, smallest_value_sensor2;

    //Serial.println("Looking for a glass");
    for (i = 0; i < num_scans; i++)
    {
        scan(50, 50, 0, sensor1, sensor_table1, scan_array1, length_sensor_table, i);
        delay(50);
    }

    //Serial.println("Changing scanner");
    delay(1000);
    for (i = 0; i < num_scans; i++)
    {
        scan(50, 50, 1, sensor2, sensor_table2, scan_array2, length_sensor_table, i);
        delay(50);
    }
    smallest_value_sensor1 = find_smallest_distance(scan_array1);
    smallest_value_sensor2 = find_smallest_distance(scan_array2);

    /*Serial.println("-------");
    Serial.println(num_scans - smallest_value_sensor2);
    Serial.println(smallest_value_sensor1);
    Serial.println("--------");*/

    if (scan_array1[num_scans - smallest_value_sensor2].distance != 0)
    //if (scan_array1[smallest_value_sensor1].distance != 100)
    {
        //avance la distance donnee par un des capteur du haut
        //vide pompe
        //retourne a la position initiale
        //Serial.println("Found a glass");
        for (i = 0; i < MAXSCAN - smallest_value_sensor2 - 1; i++)
        {
            turn(50, 50, 0);
            delay(150);
        }
    }
    else
    {
        //Serial.println("No glass found");
        //tourner a gauche ou a droite et refaire le scan jusqace qu'il trouve un verre
    }
    return 0;
}

int find_plate(struct scanner array1[], struct scanner array2[], int sensor_table1[], int sensor1, int sensor2,
               int sensor_table2[], int length_sensor_table, int num_scans)
{
    int i, smallest_value_sensor1, smallest_value_sensor2;

    //Serial.println("Looking for a plate");
    for (i = 0; i < num_scans; i++)
    {
        scan(50, 50, 0, sensor1, sensor_table1, scan_array1, length_sensor_table, i);
        delay(50);
    }

    //Serial.println("Changing scanner");
    delay(1000);
    for (i = 0; i < MAXSCAN; i++)
    {
        scan(50, 50, 1, sensor2, sensor_table2, scan_array2, length_sensor_table, i);
        delay(50);
    }

    smallest_value_sensor1 = find_smallest_distance(scan_array1);
    smallest_value_sensor2 = find_smallest_distance(scan_array2);

    if (smallest_value_sensor1 != 0 &&
        scan_array2[MAXSCAN - smallest_value_sensor1].distance == 100)
    {
        //avance la distance donnee par un des capteur
        //Serial.println("Plate found");
        for (i = 0; i < smallest_value_sensor1 - 1; i++)
        {
            turn(50, 50, 0);
            delay(150);
        }
    }
    else
    {
        //Serial.println("No plate found");
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
 *                                                Distributeur à pain                                                            * 
*********************************************************************************************************************************/

void Distributeur_Pain(int nbr_pulse, float vitesse)
{
    for(int i =0; i < 4;i++){
        sortirPain(30000);
    }
        sortirPain(1500);
        delay(1000);
    for(int i =0; i < 4;i++){
        ramenerPain(30000);
    }
        ramenerPain(4900);
}

void ramenerPain(int nbr_pulse)
{
    for(int i = 0; i < (int)nbr_pulse/3200;i++)
    {
        while (ENCODER_Read(droit) >= -3200)
        {
            MOTOR_SetSpeed(droit, -1);
        }
        ENCODER_Reset(droit);
    }

    while(ENCODER_Read(droit) >= nbr_pulse%3200)
    {
        MOTOR_SetSpeed(droit, -1);
    }
    ENCODER_Reset(droit);
    MOTOR_SetSpeed(droit, 0);
}

void sortirPain(int nbr_pulse)
{  
    Serial.println(nbr_pulse/3200);
    for(int i = 0; i < (int)nbr_pulse/3200;i++)
    {
        while (ENCODER_Read(droit) <= 3200)
        {
            MOTOR_SetSpeed(droit, 1);
        }
        ENCODER_Reset(droit);
    }

    while(ENCODER_Read(droit) <= nbr_pulse%3200)
    {
        MOTOR_SetSpeed(droit, 1);
    }
    ENCODER_Reset(droit);
    MOTOR_SetSpeed(droit, 0);
}