#include "fonctionMouvement.h"

void setup()
{
    BoardInit();
    pinMode(output_pin, OUTPUT);
    pinMode(test_pin, OUTPUT);
    Serial.begin(9600);
}

char message[] = "69";
char messageTemp[] = "69";

void loop()
{
    if (Serial.available())
    {
        getBluetooth(message);
        message[2] = '\0';
    }

    if (messageTemp != message)
    {
        messageTemp[0] = message[0];
        messageTemp[1] = message[1];

        //suiveur jusqu'a la ligne complete
        PIDSuiveurLigne();

        if (message[0] == 'X')
        {
            //pas de toast
        }
        else
        {
            //toast
        }

        if (message[1] == 'X')
        {
            //pas de jus
        }
        else
        {
            //jus
        }
    }
}
