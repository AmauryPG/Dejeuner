#include "fonctionTraitement.h"

/*********************************************************************************************************************************
 *                                                         General                                                               * 
*********************************************************************************************************************************/

//retourne la distance en encodeur
int getDistanceEncodeur(float distanceEnCM)
{
    //formule pour transfomer CM pour encodeur (ratio ; 7,8cm : 1tour ; 3200pulse : 1tour)
    return (3200 / (PI * 7.8) * distanceEnCM) + 20;
}

//modifier legerement la variable de rotation vers 19, pas trop
//retourne l'angle(degre) en encodeur d'arc
int getAngleEncodeur(float angleEnDegre)
{
    //calcule pour avoir l'arc et ensuite le transfomer en language encodeur
    return getDistanceEncodeur((19.4) * PI * angleEnDegre / 360);
}

/*********************************************************************************************************************************
 *                                                        Bluetooth                                                              * 
*********************************************************************************************************************************/

void getBluetooth(char message[])
{
    int index = 0;
    if (Serial.available())
    {
        while (Serial.available() > 0)
        {
            message[index] = Serial.read();
            index++;
        }
        message[index] = '\0'; 
    }
}

/*********************************************************************************************************************************
 *                                                         Aligneur                                                              * 
*********************************************************************************************************************************/

int getDistanceInfrarouge(int location)
{
    int distance = 4800 / (ROBUS_ReadIR(location) - 20);
    if (distance > 10 && distance < 60)
    {
        return distance;
    }
    else
    {
        return 1;
    }
}

int add_distance_to_table(int location, int table[], int length)
{
    int i;
    for (i = 0; i < length; i++)
    {
        table[i] = getDistanceInfrarouge(location);
    }
    return 0;
}

int make_histogram(int histogram[], int value_table[], int length_table, int length_histo)
{
    int i;
    int value;
    for (i = 0; i < length_histo; i++)
    {
        histogram[i] = 0;
    }

    for (i = 0; i < length_table; i++)
    {
        value = value_table[i];
        histogram[value] += 1;
    }
    return 0;
}

int get_most_read_distance(int histogram[], int length)
{
    int i;
    int most_read = 0;
    for (i = 0; i < length; i++)
    {
        if (i == 0)
        {
            most_read = i;
        }
        if (histogram[i] > most_read)
        {
            most_read = i;
        }
    }
    return most_read;
}

int distance_in_cm(int location, int sensor_table[], int length)
{
    int histo_length = 80;
    int histogram[histo_length];
    add_distance_to_table(location, sensor_table, length);
    make_histogram(histogram, sensor_table, length, histo_length);
    int distance = get_most_read_distance(histogram, histo_length);
    //  Serial.println(distance);
    return distance;
}

int check_distance(int max_distance, int sensor, int table[], int length)
{
    int distance_to_object;
    distance_to_object = distance_in_cm(sensor, table, length);
    //  Serial.println(distance_to_object);
    if (distance_to_object <= max_distance && distance_to_object > 5)
    {
        return distance_to_object;
    }
    else
    {
        return 100;
    }
}

