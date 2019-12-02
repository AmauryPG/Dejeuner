#include "fonctionMouvement.h"

void setup()
{ 
    BoardInit();
    Serial.begin(9600);
    pinMode(DP_pin1,INPUT);
    pinMode(DP_pin2,INPUT);
}
int i = 0;
void loop()
{
    //MOTOR_SetSpeed(droit,-1); 
    //MOTOR_SetSpeed(droit, 0);
    //  Serial.println("---------");
    //  Serial.println(digitalRead(DP_pin1));
    //  Serial.println(digitalRead(DP_pin2));
    //  Serial.println("---------");
    // delay(10);
    //Serial.println(digitalRead(DP_pin2));
    //MOTOR_SetSpeed(droit,-1);
    //    Serial.println(ENCODER_Read(droit));
    //MOTOR_SetSpeed(droit,1);

    Distributeur_Pain(0,0); 
    delay(10000);
    
}