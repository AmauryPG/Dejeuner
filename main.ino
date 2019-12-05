#include "fonctionMouvement.h"
#include "fonctionTraitement.h"


/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programme
// -> Se fait appeler seulement un fois
// -> Generalement on y initilise les varibbles globales

/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

/*
void loop() 
{
//find_glass(scan_array1, scan_array2, values_sensor0,  0 , 1, values_sensor2, MAXVALUES, MAXSCAN);
//delay(10000);
  /*
  find_plate(scan_array1, scan_array2, values_sensor0, 0, 1, values_sensor1, MAXVALUES, MAXSCAN);
  /*
  int i;
  for (i=0;i<MAXSCAN;i++)
  {
    scan(50, 50, 0, 0, values_sensor0, scan_array1, MAXVALUES, i);
    delay(50);
  }
  Serial.println("Changing scanner");
  delay(1000);
  for (i=0;i<MAXSCAN;i++)
  {
    scan(50, 50, 1, 1, values_sensor1, scan_array2, MAXVALUES, i);
    delay(50);
  }
  delay(2000);
  int smallest_value_sensor1 = find_smallest_distance(scan_array1);
  int smallest_value_sensor2 = find_smallest_distance(scan_array2);
  Serial.println(smallest_value_sensor1);
  Serial.println(smallest_value_sensor2);
  if (smallest_value_sensor2 != 0)
    for (i=0;i<smallest_value_sensor1-1;i++)
    {
      turn(50, 50, 0);
      delay(150);
    }
  else
  {
    Serial.println("plate");
  }
  //delay(10000);
  //getDistanceInfrarouge(0);
  //Serial.println(distance_in_cm(1, values_sensor1, MAXVALUES));
  //Serial.println(check_distance(35, 0, values_sensor0, MAXVALUES));
  delay(1000);
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU
*/
void setup()
{
  BoardInit();
  SuiveurLigneInit();
  pinMode(pin_pompe, OUTPUT); /*Pin pompe */
  pinMode(output_pin, OUTPUT);
  pinMode(test_pin, OUTPUT);
  Serial.begin(9600);
  SERVO_Enable(0);
//  SERVO_SetAngle(0, 114);
//  delay(1000);
//  SERVO_Disable(0);
  
}

char message[] = "69";

void loop()
{
    
    monterPelle(180);
    delay(2000);
  if (Serial.available())
  {
    getBluetooth(message);
    Serial.println(message);

    if ((message[0] == 'T' || message[0] == 'X') && (message[1] == 'X' || message[1] == 'J'))
    {
      if (message[0] == 'X' && message[1] == 'X')
      {
        Serial.println("PAS DE DEJEUNER");
      }
      else
      {
        SuiveurLigneArret();
        //sequence pour la toast
        if (message[0] == 'T')
        {
            Serial.println("Toast");
            avancer(8, 0.2);
            SuiveurLigneArret();
            avancer(3, 0.2);
            tournerSuiveurLigneGaucher();
            cuitLaToast();
        }
       
        else
        {
          //pas de sequence pour le dispenser
          //arriver au jus
          avancer(3, 0.2);
          SuiveurLigneArret();
          avancer(3, 0.2);
          tournerSuiveurLigneGaucher();
        }
        avancer(3, 0.2);
        SuiveurLigneArret();
        avancer(3, 0.2);
        tournerSuiveurLigneGaucher();

        SuiveurLigneArret();

        /////////////////////////////////////////////////////////////////

        if (message[1] == 'J')
        {
          //chercher jus
          TrouverVerre();
          videPompe();
          delay(1000);
          /////////////////////////////////////////////////////////////////
          while (getPosition == 0)
          {
            MOTOR_SetSpeed(gauche, -0.2);
            MOTOR_SetSpeed(droit, -0.2);
          }

          delay(200);

          while (getPosition() == 0)
          {
            MOTOR_SetSpeed(gauche, -0.1);
            MOTOR_SetSpeed(droit, -0.1);
          }
          MOTOR_SetSpeed(gauche, 0);
          MOTOR_SetSpeed(droit, 0);
          avancer(2, 0.2);
        }
        else
        {
          avancer(3, 0.2);
        }
        if (message[0] == 'T')
        {
          tournerSuiveurLigneGaucher();

          SuiveurLigneArret();

          avancer(3, 0.2);
          tournerDroit(0.1,0,87);


          MOTOR_SetSpeed(gauche, 0);
          MOTOR_SetSpeed(droit, 0);
          //Pogner l'assiette
          avancer(5, 0.2);
          avancer(5, 0.2);
          TrouverAssiette();
          tournerDroit(0.1, 0, 175);

          descendrePelle(20);
          cuitLaToast();
          avancer(10, 0.1);
          monterPelle(140);
          delay(10000);
        }
      }
    }
    else
    {
      Serial.println("ERROR DANS LE BLUETOOTH");
    }
  }
  
//prendreToast();
//    TrouverVerre();
//printSuiveurLigne();
//monterPelle(180);
  delay(20);
}