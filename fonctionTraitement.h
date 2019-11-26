#include <Arduino.h>
#include <LibRobus.h> 
#include <AutoPID.h>
#include <SoftwareSerial.h> 
#include <QTRSensors.h>

#define gauche 0
#define droit 1

#define pince 0

#define KP 0.1
#define KI 0.0
#define KD 0.0

#define OUTPUT_MIN -1
#define OUTPUT_MAX 1

#define MAXVALUES 10
#define MAXSCAN 25

///////////////////////////////////////////////////////General<///////////////////////////////////////////////////////

int getDistanceEncodeur(float distanceEnCM);

int getAngleEncodeur(float angleEnDegre);

///////////////////////////////////////////////////////Bluetooth//////////////////////////////////////////////////////

void getBluetooth(char message[]);

///////////////////////////////////////////////////////Aligneur///////////////////////////////////////////////////////

int getDistanceInfrarouge(int location);

int add_distance_to_table(int location, int table[], int length);

int make_histogram(int histogram[], int value_table[], int length_table, int length_histo);

int get_most_read_distance(int histogram[], int length);

int distance_in_cm(int location, int sensor_table[], int length);

int check_distance(int max_distance, int sensor, int table[], int length);
