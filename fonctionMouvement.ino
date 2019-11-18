#include "fonctionMouvement.h"

/*********************************************************************************************************************************
 *                                                          PID                                                                 *
 *                                                     *Pas toucher*                                                            *
*********************************************************************************************************************************/
/*void PIDAvancer(PID pid, float &input, float &output, float vitesse, float distanceEnCM)
{
    while (input < getDistanceEncodeur(distanceEnCM))
    {
        input = ENCODER_Read(gauche);
        pid.Compute();
        MOTOR_SetSpeed(droit,output);
        MOTOR_SetSpeed(gauche,vitesse);
    }

    MOTOR_SetSpeed(droit,0);
    MOTOR_SetSpeed(gauche,0);

    ENCODER_Read(gauche);
    ENCODER_Read(droit);
}

void PIDAvancer(PID pid, float &input, float &output, float vitesseInitiale, float vitesseFinale, float distanceEnCM, float distanceAccelerationEnCM)
{
    PIDAcceleration(pid, input, output,vitesseInitiale,vitesseFinale,distanceAccelerationEnCM);
    PIDAvancer(pid,input,output,vitesseFinale,distanceEnCM);
}

void PIDAvancer(PID pid, float &input, float &output, float vitesseInitiale, float vitesseintermediaire, float vitesseFinale,
                     float distanceEnCM, float distanceAccelerationEnCM, float distanceDecelerationEnCM)
{
    PIDAcceleration(pid, input, output, vitesseInitiale, vitesseintermediaire, distanceAccelerationEnCM);
    PIDAvancer(pid, input, output, vitesseintermediaire, distanceEnCM);
    PIDAcceleration(pid, input, output, vitesseintermediaire, vitesseFinale, distanceDecelerationEnCM);
}

void PIDAcceleration(PID pid, float &input, float &output, float vitesseInitial, float vitesseFinale, float distanceCM)
{
    
     * x   | y
     * 100 | 109
     * 95  | 53
     * 90  | 52
     * 70  | 50
     * 50  | 50
     * 25  | 48
    
    float constante = ((-24.5407) / (distanceCM - 100.408)) + 48.8973;
    float acceleration = constante * ((pow(vitesseFinale, 2) - pow(vitesseInitial, 2)) / (2 * getDistanceEncodeur(distanceCM)));
    int temps = 1;
    float vitesseModifier = vitesseInitial + acceleration;

    while (vitesseModifier <= vitesseFinale)
    { 
        MOTOR_SetSpeed(droit,vitesseModifier);
        pid.Compute();
        MOTOR_SetSpeed(output,vitesseModifier);

        temps++;
        vitesseModifier = vitesseInitial + acceleration * temps;
    }

    ENCODER_Reset(gauche);
    ENCODER_Reset(droit);
}*/