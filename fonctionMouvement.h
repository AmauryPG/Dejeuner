#include "fonctionTraitement.h"
#define DP_pin1 30
#define DP_pin2 41

/*********************************************************************************************************************************
 *                                                          PID                                                                 *
 *                                                     *Pas toucher*                                                            *
*********************************************************************************************************************************/

void SuiveurLigneInit();

void PIDSuiveurLigne();

/*********************************************************************************************************************************
 *                                                         Aligneur                                                              * 
*********************************************************************************************************************************/

void TrouverVerre();

void TrouverAssiette();

int find_smallest_distance(struct scanner array[]);

/*********************************************************************************************************************************
 *                                                          Pelle                                                                * 
*********************************************************************************************************************************/

void actionPain();

void PainInit();

/*********************************************************************************************************************************
 *                                                Distributeur à pain                                                            * 
*********************************************************************************************************************************/

void Distributeur_Pain(int nbr_pulse, float vitesse);
