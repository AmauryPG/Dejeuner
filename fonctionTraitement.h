#include <Arduino.h>
#include <LibRobus.h> 
#include <AutoPID.h> 
#include <PID_AutoTune_v0/PID_AutoTune_v0.h>
#include <QTRSensors.h>

#define gauche 0
#define droit 1

#define pince 0

int getDistanceEncodeur(float distanceEnCM);

int getAngleEncodeur(float angleEnDegre);