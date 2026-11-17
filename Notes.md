
Arduino Pin :

 * PIN_STEP    = 2;    // CNC Shield axe Y : STEP
 * PIN_DIR     = 5;    // CNC Shield axe Y : DIR
 * PIN_ENABLE  = 4;    // CNC Shield : EN (actif à LOW)
 * PIN_ENDSTOP = 9;   // A AJUSTER : Y- du CNC Shield = D10 (pin non précisée)
 * PIN_PUMP[4] = {23, 25, 27, 29};
 * PIN_GLASS   = A9;   // RP-S40-ST
 * PIN_LED     = 22;   // WS2812B
 * PIN_SERVO   = A8;   // HS-422

********************************************

Commandes : 

HOME            	* Homing rapide, puis affiche la position
POS	                * Position actuelle (cm et pas depuis HOME)
GOTO:x	            * Va à x cm (ex : GOTO:10, GOTO:80)
JOG:d	            * Déplacement relatif en cm (ex : JOG:0.5, JOG:-0.2)
JUICE           	* Va au Soft (25 cm)
DISP:1 à DISP:6	    * Va au distributeur Hard 1 à 6
TOUR	            * HOME, puis jus et les 6 Hard avec 4 s d’arrêt chacun, puis retour à 0
STOP	            * Interrompt le déplacement, la pompe ou la préparation en cours

PUMP:n:ms	        * Pompe n (1 à 4) pendant ms millisecondes, 30000 au maximum
PUMPML:n:ml     	* Pompe n pendant la durée calculée pour ce volume, 300 ml au maximum

STATUS	            * État (IDLE, BUSY, DONE, ERROR), verre détecté ou non, homing fait ou non
SENSOR	            * Valeur brute du détecteur de verre, pour régler GLASS_THRESHOLD
1 à 22, C3 ou COCKTAIL:12

SERVO:angle	        * Place le servo à l’angle donné (0 à 180), le tient 1 s, puis le relâche. Exemple : SERVO:60.
DOSE:n	            * Verse n doses (1 à 5) à l’endroit où se trouve le chariot. Il pousse, attend l’écoulement, puis laisse le doseur se remplir.

***********************************************************

Distributeurs :

Soft 1 Orange
Soft 2 Ananas
Soft 3 Cranberry
Soft 4 Pamplemousse

Hard 1 Rhum blanc
Hard 2 Whisky
Hard 3 Tequila
Hard 4 Vodka
Hard 5 Curaçao bleu
Hard 6 Amaretto

***********************************************************
