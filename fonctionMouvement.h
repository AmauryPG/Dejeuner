#include "fonctionTraitement.h"
#include "LibRobus.h"


/*********************************************************************************************************************************
 *                                                          General                                                              * 
*********************************************************************************************************************************/

void avancer(float distance, float vitesse);

void reculer(float distance, float vitesse);

void tournerDroit(float vitesse, int offset, float angle);

void tournerGauche(float vitesse, int offset, float angle);

void tournerSuiveurLigneGaucher();

void tournerSuiveurLigneDroit();

/*********************************************************************************************************************************
 *                                                          PID                                                                 *
 *                                                     *Pas toucher*                                                            *
*********************************************************************************************************************************/

void SuiveurLigneInit();

int SuiveurLigneInverser();

int SuiveurLigne();

void printSuiveurLigne();

int getPosition();

void SuiveurLigneArret();

/*********************************************************************************************************************************
 *                                                         Aligneur                                                              * 
*********************************************************************************************************************************/

void TrouverVerre();

void TrouverAssiette();

int find_smallest_distance(struct scanner array[]);

void cuitLaToast();

/*********************************************************************************************************************************
 *                                                          Pelle                                                                * 
*********************************************************************************************************************************/

void prendreToast();

void descendrePelle(int angle);

void monterPelle(int angle);

void deposeToast();



/*********************************************************************************************************************************
 *                                                         Pompe                                                               * 
*********************************************************************************************************************************/

void videPompe();
