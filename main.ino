#include "fonctionTraitement.h"

void setup()
{
    BoardInit();
    Serial.begin(9600);
}

void loop()
{
    char message[5];

    getBluetooth(message);

    Serial.println(message);
    delay(100);
}