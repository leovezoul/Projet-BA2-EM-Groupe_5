/*
Code permettant au robot de recevoir la case jouée par l'adversaire grâce à une radio, en déduire grâce 
à l'algorithme minimax la meilleure case à joueur. Et s'y rends ensuite grâce à ses capteurs IR.
 */

// Bibliothèques
#include <Arduino.h>
#include <math.h>
#include <QTRSensors.h>
#include <Servo.h>
#include <SPI.h>
#include <RF24.h>







// Sélection joueur
bool jeJoueEnPremier = false;

// Paramétrage de la radio
RF24 radio(48,49); 
const byte addresses[][6] = {"1Node", "2Node"};

// Mémoires de jeu
int board[9] = {0};
int moves = 0; 
int dernierCoupJoue = -1;

// Initialisation des états de jeu
enum EtatJeu { INITIALISATION, ATTENTE_ADVERSAIRE, MON_TOUR_CALCUL, EN_DEPLACEMENT, FIN_PARTIE };
EtatJeu etatJeu = INITIALISATION;

// Cas dépendants de la case à jouer
int directionX = 0; // =1 si le robot va vers la droite du plateau, =-1 si à gauche 
int cibleActuelle = 0; // case que le robot joue









// Vitesse du robot
const float VITESSE_BASE = 0.15f;

// Pins des moteurs
const uint8_t in1Pins[4] = {22, 24, 26, 28}; ///< Pins directionnels moteurs
const uint8_t in2Pins[4] = {23, 25, 27, 29}; ///< Pins directionnels moteurs
const uint8_t pwmPins[4] = {2, 3, 4, 5};     ///< Pins de puissance PWM

//Pins du servo 
const int servoPin = 9;
const int SERVO_UP = 50;
const int SERVO_DOWN = 200;
const unsigned long SERVO_TIME_MS = 300; 
Servo styloServo;

// Pins des capteurs
const uint8_t SensorCount = 5;
const uint8_t sensorPinsHoriz[SensorCount] = {A4, A3, A2, A1, A0}; // Pins du capteur horizontal (lorsque le robot avance tout droit)
const uint8_t sensorPinsVert[SensorCount]  = {A9, A8, A7, A6, A5}; // Pins du capteur vertical (lorsque le robot roule latéralement)

// Variables de type QTRSensors utilisés par la bibliothèque polulu et qui lit les pins au dessus
QTRSensors qtrHoriz;
QTRSensors qtrVert; 

uint16_t valuesHoriz[SensorCount]; 
uint16_t valuesVert[SensorCount]; 

// Seuils auxquels les capteurs voient du blanc et du noir, peut être modifié selon les besoins
const uint16_t SEUIL_NOIR = 700; 
const uint16_t SEUIL_BLANC = 300; 

// Valeurs proportionnelles et différentielles
/* y : lorsque le robot roule tout droit dans la grille
   x : lorsque le robot roule latéralement dans la grille
   Coefficients différentiels et proportionnels de correction latérale et 
   de rotation*/ 
float Kp_y = -0.00015f; 
float Kd_y = 0.0f; 
float Kp_roty = -0.00026f;
float Kd_roty = 0.0f; 
float lastErrorHoriz = 0.0f; 

float Kp_x = -0.00015f; 
float Kd_x = 0.0f; 
float lastErrorVert = 0.0f; 
float Kp_rotx = -0.00026f; 
float Kd_rotx = 0.0f; 

/* Initialisation des compteurs. Compteur ligne lorsque le robot avance tout droit, 
Compteur colonne lorsque le robot roule latéralement */
int compteurLigne = 0; 
int objLigne = 0; 
int compteurColonnes = 0; 
int objColonne = 0;  

// Assistances et sécurités de comptage de ligne
bool surIntersectionPrecedente = false; // Vrai si le robot est toujours sur la même ligne, pour ne pas compter plusieurs fois la même ligne
unsigned long dernierTempsCroisement = 0; // Dernière fois que le robot a vu une ligne noire, est utilisé avec DELAI_ANTI_REBOND
const unsigned long DELAI_ANTI_REBOND = 500; // Donne du temps au robot de s'éloigner

unsigned long tempsDebutMouvement = 0; 
const unsigned long DELAI_ZONE_AVEUGLE = 500; // delai permettant de passer une ligne sans la compter si le robot est allé trop loin

// Etats du déplacement 
enum EtatPhysique { EN_ATTENTE, TRAJET_ALLER, GLISSADE_ALLER, DESSIN, RETOUR_CENTRE, RETOUR_BASE }; // Enumeration
EtatPhysique etatPhysique = EN_ATTENTE; // etatphysique == 0
EtatPhysique dernierEtatPhysiqueAffiche = EN_ATTENTE; // Mémoire état physique, permettre d'afficher l'état dans la console une seule fois








/* Pour chaque moteur la fonction prend comme argument la vitesse à laquelle le moteur doit tourner 
et trouve le PWM à appliquer pour atteindre cet objectif
idx est le moteur et motor_rps est la vitesse demandée en rotation par seconde  */
void setMotorPWMDir(int idx, float motor_rps) {

    bool dir_forward = (motor_rps >= 0.0f); // Vrai si le robot va vers l'avant

    float abs_rps = fabs(motor_rps); // Vitesse absolue demandée

    float command = abs_rps / 3.83f; // On transforme la vitesse demandée en pourcentage de la vitesse max

    if (command > 1.0f) command = 1.0f; /*Sécurité qui limite la vitesse à 100% de la vitesse max 
                                        (sinon le code pourrait demander un PWM > 255)*/

    int pwm_val = (int)round(command * 255); // On change l'échelle vers 0 à 255 pour arduino

    digitalWrite(in1Pins[idx], dir_forward ? HIGH : LOW); // Communique au driver le sens de rotation en fonction de la direction de déplacement 
    digitalWrite(in2Pins[idx], dir_forward ? LOW : HIGH);
    analogWrite(pwmPins[idx], pwm_val); // Communique au driver la vitesse de rotation des moteurs
}

/* Traduit la vitesse physique du robot vers les vitesses de chaque moteur
    Vx vitesse dans le sens x
    Vy vitesse dans le sens y (perpendiculaire à x)
    omega vitesse angulaire */
void setRobotSpeed(float vx, float vy, float omega) {
    const float alpha_arr[4] = {PI/4.0f, 3.0f*PI/4.0f, 5.0f*PI/4.0f, 7.0f*PI/4.0f}; // Angles géométriques de chaque roue écris en radiant 
                                                                                    // (45°, 135°, 225°, 315°)

    const float correction[4] = {1.0f,1.042f,1.236f,1.134f}; // facteurs de correction corrigeant les erreures physiques

    float phi_dot[4]; // Pour les 4 roues


    for (int i = 0; i < 4; ++i) {
        float numer = -sin(alpha_arr[i]) * vx + cos(alpha_arr[i]) * vy + 0.1378f * omega; // calcule la vitesse linéaire (m/s) de chaque roue pour accomplir le mouvement global
                                                                                          // 0.1378 distance roue/centre de rotation en mètres
        phi_dot[i] = numer / 0.04f; // Convertit vitesse m/s vers radiants/s
        setMotorPWMDir(i, (phi_dot[i] / (2.0f * PI)) * correction[i]); /* On convertir en tour/s pour pouvoir utiliser la fonction
                                                                       définie juste avant et qui calcule le PWM pour faire tourner 
                                                                       les moteurs à cette vitesse */

    } 
}

void stopmoteurs() { setRobotSpeed(0, 0, 0); } // Fonction vitesse nulle








// Calcul de l'erreur de centrage du capteur central
float getErreurHoriz() {
    uint16_t position = qtrHoriz.readLineBlack(valuesHoriz); // Fonction qui renvoie un nombre entre 0 et 4000 selon où est la ligne noire
    

    //Sécurité : Si aucune capteur ne voit du noir, le robot soit continuer tout droit, sans cette partie le robot 
    // aurait un comportement indéfini dans ces cas là
    bool toutBlanc = true; 
    for (uint8_t i=0; i<SensorCount; i++) {
        if(valuesHoriz[i]>SEUIL_BLANC) { 
            toutBlanc=false; break; }
    } 
    if (toutBlanc) {
        return 0.0f; 
    } // On considère que l'erreur est nulle si tous les capteurs voient du blanc pour que le robot ne corrige pas


    return (float)position - 2000.0f; /* L'erreur est la différence de position entre le centre et le capteur sur la ligne noire
                                        2000 étant la position du capteur central*/

}


// Calcul de l'erreur de centrage du capteur vertical, la logique est exactement la même que pour le capteur horizontal
float getErreurVert() {
    uint16_t position = qtrVert.readLineBlack(valuesVert);
    bool toutBlanc = true;
    for (uint8_t i=0; i<SensorCount; i++) if(valuesVert[i]>SEUIL_BLANC) { toutBlanc=false; break; }
    if (toutBlanc) return 0.0f; 
    return (float)position - 2000.0f; 
}


/* Détection d'intersection pour le compteur horizontal, les deux capteurs extrêmes doivent voir du noir en même temps
pour compter l'intersection*/
bool horizSurLigneNoire() { return (valuesHoriz[0] > SEUIL_NOIR && valuesHoriz[4] > SEUIL_NOIR); } // Pour la condition de croisement 
bool horizSurBlanc()      { return (valuesHoriz[0] < SEUIL_BLANC && valuesHoriz[4] < SEUIL_BLANC); } // Pour la condition de fin de croisement



/* Détection d'intersection pour le compteur vertical. Des excecptions doivent être faites au vu de la géométrie de 
déplacement pour que les capteurs puissent compter les lignes*/
bool vertSurLigneNoire()  { 

    /* La position des cases change selon si on commence ou pas, il faut donc instaurer des conditions car 
    dans ce cas les cases ciblées par la condition sont les cases les plus lointaines de la zone de départ du robot */
    if(jeJoueEnPremier) {
        if (cibleActuelle == 3 || cibleActuelle == 6 || cibleActuelle == 9) { 
            return (valuesVert[3] > SEUIL_NOIR && valuesVert[4] > SEUIL_NOIR); 
        } /* Le robot ne se sert plus des deux capteurs extrêmes pour le comptage de ligne car dans ce cas il n'y a jamais
            de ligne noire sous le capteur 0 */
        else {
            return (valuesVert[0] > SEUIL_NOIR && valuesVert[4] > SEUIL_NOIR);
        } // Comptage normal
    }


    else {
        if (cibleActuelle == 1 || cibleActuelle == 4 || cibleActuelle == 7) { 
            return (valuesVert[3] > SEUIL_NOIR && valuesVert[4] > SEUIL_NOIR); 
        } 
        else {
            return (valuesVert[0] > SEUIL_NOIR && valuesVert[4] > SEUIL_NOIR);
        }
    } // Même logique que quand le robot joue en premier

} 
// Conditions pour considérer être sorti de l'intersection, même logique que pour les lignes noires 
bool vertSurBlanc()  { 
    if(jeJoueEnPremier) {
        if (cibleActuelle == 3 || cibleActuelle == 6 || cibleActuelle == 9) { 
            return (valuesVert[3] < SEUIL_BLANC && valuesVert[4] < SEUIL_BLANC); 
        } 
        else {
            return (valuesVert[0] < SEUIL_BLANC && valuesVert[4] < SEUIL_BLANC);
        }
    }
    else {
        if (cibleActuelle == 1 || cibleActuelle == 4 || cibleActuelle == 7) { 
            return (valuesVert[3] < SEUIL_BLANC && valuesVert[4] < SEUIL_BLANC); 
        } 
        else {
            return (valuesVert[0] < SEUIL_BLANC && valuesVert[4] < SEUIL_BLANC);
        }
    }
} 



// Dessine une grille OXO
void draw_board(int *b) {
    for(int i=0; i<9; i+=3) {
        Serial.print(b[i]); Serial.print("|"); 
        Serial.print(b[i+1]); Serial.print("|"); 
        Serial.println(b[i+2]);
    }
}


// Vérification des conditions de victoire, regardé à chaque fois que le robot joue
int win(const int *board) {
    unsigned wins[8][3] = {{0,1,2},{3,4,5},{6,7,8},{0,3,6},{1,4,7},{2,5,8},{0,4,8},{2,4,6}}; // Toutes les combinaisons victorieuses
    for (int i = 0; i < 8; ++i) {

        // Regarde tour à tour si une des différentes combinaison est remplie par le même symbole
        if (board[wins[i][0]] != 0 && board[wins[i][0]] == board[wins[i][1]] && board[wins[i][1]] == board[wins[i][2]])

            return board[wins[i][2]]; //Joueur qui a gagné 
    }
    return 0; // Personne n'a encore gagné 
}


// Regarde si 
int minimax(int *board, int player) {
    // 1. On vérifie si quelqu'un a déjà gagné dans cette simulation
    int winner = win(board);
    
    // 2. Si un gagnant existe (1 ou -1), on multiplie par 'player' pour savoir qui a gagné
    if (winner != 0) return winner * player;

    // 3. 'move' stockera la meilleure case, 'score' est initialisé très bas (-2)
    // pour être sûr que n'importe quel score trouvé sera meilleur
    int move = -1, score = -2;

    // 4. On teste toutes les cases de la grille (0 à 8)
    for (int i = 0; i < 9; i++) {
        
        // 5. Si la case est vide, on tente de jouer dedans
        if (board[i] == 0) {
            
            // 6. On pose virtuellement le pion du joueur actuel
            board[i] = player;

            // 7. RÉCURSIVITÉ : Le robot regarde ce qui se passe si l'adversaire joue après
            // On change de joueur (player * -1) et on inverse le score (-) 
            // car ce qui est bon pour l'adversaire est mauvais pour nous.
            int thisScore = -minimax(board, player * -1);

            // 8. Si ce coup simulé donne un meilleur résultat que les précédents, 
            // on le mémorise comme étant notre nouveau "meilleur coup"
            if (thisScore > score) { 
                score = thisScore; 
                move = i; 
            }


            // remettre la grille dans l'état d'origine avant de tester la case suivante
            board[i] = 0;
        }
    }

    // 10. Si 'move' est resté à -1, c'est que la grille est pleine (Match nul -> 0)
    // Sinon, on renvoie le score du meilleur coup trouvé
    return (move == -1) ? 0 : score;
}


// Calcule la meilleure case à jouer
int computerMove(int *board) {
    // 1. Initialisation : 'move' est la case choisie (0 à 8), 
    // 'score' est mis à -2 (très bas) pour être sûr de trouver mieux.
    int move = -1, score = -2;

    // 2. On parcourt les 9 cases de la grille pour trouver lesquelles sont vides.
    for (int i = 0; i < 9; ++i) {
        
        // 3. Si la case 'i' est libre (égale à 0)
        if (board[i] == 0) {
            
            // 4. On fait un test : "Et si le robot (joueur 1) jouait ici ?"
            board[i] = 1;

            // 5. On demande à l'algorithme Minimax d'analyser TOUTES les conséquences.
            // On passe '-1' car c'est maintenant au tour de l'humain de répondre dans la simulation.
            int tempScore = -minimax(board, -1);

            // 6. On efface notre test pour ne pas tricher sur la vraie grille.
            board[i] = 0;

            // 7. Si ce coup donne un meilleur score que les autres tests précédents,
            // on le retient comme étant le meilleur choix actuel.
            if (tempScore > score) { 
                score = tempScore; 
                move = i; 
            }
        }
    }

    // 8. Une fois qu'on a testé toutes les cases vides, on renvoie l'indice (0-8)
    // de la meilleure case possible.
    return move;
}







// transmets coup à l'adversaire par radio
void envoyerCoupRF(int coup) {
    radio.stopListening(); // Le robot doit arrêter d'écouter pour envoyer
    radio.write(&coup, sizeof(int));  // Va voir l'adresse de la variable coup et envoie, on lui dit combien d'octets elle doit copier
    radio.startListening(); // Elle a transmis, elle peut écouter
}







// fonction de dessin du rond
void dessinrond() { 
    setRobotSpeed(0.0f, 0.0f, -1.5f); // robot pivote sur lui-même
    delay(3000); // Continue à pivoter pendant 3 secondes
    stopmoteurs(); // S'arrête
}


// Fonction de dessin de la croix 
void dessincroix() {
    if ((jeJoueEnPremier && (cibleActuelle == 7 || cibleActuelle == 8 || cibleActuelle == 9)) || (!jeJoueEnPremier && (cibleActuelle == 1 || cibleActuelle == 2 || cibleActuelle == 3))) {
        
        setRobotSpeed(-0.35, 0.0f, 0.0f); delay(100); stopmoteurs(); delay(500);
        setRobotSpeed(0.35, 0.0f, 0.0f); delay(634); stopmoteurs(); delay(500);
        setRobotSpeed(-0.17, 0.17, 0.0f); delay(612); stopmoteurs(); delay(500);
        setRobotSpeed(0.0f, -0.35, 0.0f); delay(634); stopmoteurs(); delay(500);
        setRobotSpeed(0.0f, 0.35, 0.0f); delay(317); stopmoteurs(); delay(500);
        setRobotSpeed(-0.35, 0.0f, 0.0f); delay(288); stopmoteurs();
    } // Dans ces cas précis, la configuration physique robot ne lui permet pas de faire une croix de taille normale (25x25cm)
      // Le robot fait donc une plus petite croix (20x20cm)

    else {
        setRobotSpeed(-0.35, 0.0f, 0.0f); delay(200); stopmoteurs(); delay(500);
        setRobotSpeed(0.35, 0.0f, 0.0f); delay(705); stopmoteurs(); delay(500);
        setRobotSpeed(-0.17, 0.17, 0.0f); delay(680); stopmoteurs(); delay(500);  
        setRobotSpeed(0.0f, -0.35, 0.0f); delay(705); stopmoteurs(); delay(500);
        setRobotSpeed(0.0f, 0.35, 0.0f); delay(353); stopmoteurs(); delay(500);
        setRobotSpeed(-0.35, 0.0f, 0.0f); delay(319); stopmoteurs();
    } // Fonction qui fait la croix de 25x25cm
}  

// On prépare le déplacement en initialisant les compteurs en fonction de la croix à rejoindre
void configurerDeplacementPhysique(int caseIndex) {

    // On remet à zéro les compteurs à zéro lorsqu'on prépare un nouveau coup, on ne veut pas garder le compteur
    // du coup précédent
    compteurLigne = 0; 
    compteurColonnes = 0;

    
    cibleActuelle = caseIndex; //Case à accéder


    
    // Détermination du nombre de lignes à croiser pour aller à la case. Dépend du côté par lequel on commence
    if (caseIndex == 1 || caseIndex == 4 || caseIndex == 7) objLigne = (jeJoueEnPremier) ? 3 : 5;
    else if (caseIndex == 2 || caseIndex == 5 || caseIndex == 8) objLigne = 4;
    else if (caseIndex == 3 || caseIndex == 6 || caseIndex == 9) objLigne = (jeJoueEnPremier) ? 5 : 3;

    // Détermination du nombre de colonnes à croiser pour aller à la case. Dépend du côté par lequel on commence
    if (caseIndex == 4 || caseIndex == 5 || caseIndex == 6) { objColonne = 0; directionX = 0; }
    else if (caseIndex == 7 || caseIndex == 8 || caseIndex == 9) { objColonne = 1; directionX = (jeJoueEnPremier) ? 1 : -1; }
    else if (caseIndex == 1 || caseIndex == 2 || caseIndex == 3) { objColonne = 1; directionX = (jeJoueEnPremier) ? -1 : 1; }
}


// Fonction de déplacement du robot. Lorsqu'on l'appelle, le robot va à la case, dessine, revient à la zone de départ
void gererDeplacementPhysique() {
    // Affichage propre du changement d'état physique (exécuté 1 seule fois par changement)
    // La condition du if sert à ce que l'état ne soit que écrite une fois et pas en boucle
    if (etatPhysique != dernierEtatPhysiqueAffiche) {
        dernierEtatPhysiqueAffiche = etatPhysique;
        switch (etatPhysique) {
            case TRAJET_ALLER: Serial.println(">> [ETAT] Début Trajet Y (Longitudinal)"); break;
            case GLISSADE_ALLER: Serial.println(">> [ETAT] Début Glissade X (Transversal)"); break;
            case DESSIN: Serial.println(">> [ETAT] Phase de Dessin"); break;
            case RETOUR_CENTRE: Serial.println(">> [ETAT] Début Retour X"); break;
            case RETOUR_BASE: Serial.println(">> [ETAT] Début Retour Y"); break;
            case EN_ATTENTE: Serial.println(">> [ETAT] Robot en Attente"); break;
        } 
    } // Affiche sur l'ordinateur l'état de déplacement dans lequel est le robot

    switch (etatPhysique) {
        case EN_ATTENTE: break; // Lorsque la fonction est appelée le robot quitte l'état attente


        // Le robot avance jusqu'à la ligne cible
        case TRAJET_ALLER: {

            // Tant que le compteur de ligne est inférieur à l'objectif le robot suit la ligne 
            if (compteurLigne < objLigne) {
                float err = getErreurHoriz(); // Erreur du robot

                // Sort de l'intersection
                if (horizSurBlanc() && surIntersectionPrecedente) surIntersectionPrecedente = false; 
                
                /* Condition pour compter la ligne noire. Le capteur horizontal doit voir une ligne noire
                et la dernière fois qu'il a vu une ligne noire doit être il y a plus de DELAI_ANTI_REBOND (condition pour 
                éviter que les vibrations et corrections fassent compter deux fois la même ligne) */
                if (horizSurLigneNoire() && !surIntersectionPrecedente && (millis() - dernierTempsCroisement > DELAI_ANTI_REBOND)) {
                    surIntersectionPrecedente = true; // Pour pas que le robot ne compte deux fois la même ligne
                    compteurLigne++;  // Le compteur est incrémenté
                    dernierTempsCroisement = millis(); // On commence le compteur de la dernière fois que le robot a vu une ligne noire pour la condition de comptage
                    
                    Serial.print("[TEMPS REEL] Ligne Y passée : "); 
                    Serial.print(compteurLigne); Serial.print(" / "); Serial.println(objLigne);
                 } 
            
                float corr = (Kp_y * err) + (Kd_y * (err - lastErrorHoriz)); // Calcul de la vitesse de correction
                lastErrorHoriz = err; // Pour le PID différentiel dans la prochaine boucle
                setRobotSpeed(VITESSE_BASE - corr, VITESSE_BASE + corr, (Kp_roty * err)); // Nouvelle vitesse du robot avec la correction latérale et angulaire
            } 

            // Une fois l'objectif de ligne atteint le robot s'arrête
            else {
                stopmoteurs();
                delay(400);

                // On recule le robot pendant 250ms pour le recentrer pour contrer la distance de freinage
                setRobotSpeed(-VITESSE_BASE, -VITESSE_BASE, 0.0f);
                delay(250);

                stopmoteurs();
                delay(500); 
                tempsDebutMouvement = millis();// On initialise le chronomètre pour connaitre quand a commencé le nouvel état
                etatPhysique = (objColonne == 0) ? DESSIN : GLISSADE_ALLER; // Si la case cible est au milieu le robot passe en dessin sinon en glissement
            }
            break;
        }


        // Glissement latéral vers la case cible
        case GLISSADE_ALLER: {

            // Même logique de compteur et suivi de ligne qu'avant mais avec le capteur vertical
            if (compteurColonnes < objColonne) {
                float err = getErreurVert();
                if (millis() - tempsDebutMouvement > DELAI_ZONE_AVEUGLE) {
                    if (vertSurBlanc() && surIntersectionPrecedente) surIntersectionPrecedente = false; 
                    
                    if (vertSurLigneNoire() && !surIntersectionPrecedente && (millis() - dernierTempsCroisement > DELAI_ANTI_REBOND)) {
                        surIntersectionPrecedente = true; 
                        compteurColonnes++; 
                        dernierTempsCroisement = millis();
                        
                        Serial.print("[TEMPS REEL] Colonne X passée : "); 
                        Serial.print(compteurColonnes); Serial.print(" / "); Serial.println(objColonne);
                    }
                }
                float corr = (Kp_x * err) + (Kd_x * (err - lastErrorVert)); 
                lastErrorVert = err;

                if (directionX == 1) {
                    setRobotSpeed(VITESSE_BASE + corr, -VITESSE_BASE + corr, (Kp_rotx * err));
                } // Si la case est à droite le robot se déplace vers la droite 
                else {
                    setRobotSpeed(-VITESSE_BASE + corr, VITESSE_BASE + corr, -((Kp_rotx/2) * err));
                } // Si la case est à gauche le robot va vers la gauche
            }
            else {
                stopmoteurs(); 
                delay(500);
                etatPhysique = DESSIN; // On passe à l'état dessin car le robot est arrivé à la case cible 
            }
            break;
        }


        // Le robot dessine
        case DESSIN: {
            styloServo.write(SERVO_DOWN); // Le stylo s'abaisse
            delay(SERVO_TIME_MS); // On laise au stylo le temps de s'abaisser
            if (jeJoueEnPremier) dessincroix(); // Si on joue en premier on joue la croix 
            else dessinrond(); // Sinon le cercle 
            styloServo.write(SERVO_UP); delay(SERVO_TIME_MS); // On remonte le stylo et on lui laisse le temps de remonter
            
            surIntersectionPrecedente = false;   
            dernierTempsCroisement = millis();
            tempsDebutMouvement = millis(); // on lance le chronomètre de début de l'état
            etatPhysique = (objColonne == 0) ? RETOUR_BASE : RETOUR_CENTRE; // Si on a joué dans les colonnes du milieu on passe à l'état RETOUR_BASE
                                                                            // Sinon on passe à l'état RETOUR_CENTRE
            break;
        }


        // Glissement latéral depuis la case cible vers la colonne du centre
        case RETOUR_CENTRE: {
            // Même logique qu'avant, mais cette fois ci on décrémente le compteur de colonne jusqu'à ce qu'il soit égal à 0
            // c'est à dire qu'on est revenu à la colonne du milieu
            if (compteurColonnes > 0) {
                float err = getErreurVert();
                if (millis() - tempsDebutMouvement > DELAI_ANTI_REBOND) {

                    if (vertSurBlanc() && surIntersectionPrecedente) surIntersectionPrecedente = false; 
                    if (vertSurLigneNoire() && !surIntersectionPrecedente && (millis() - dernierTempsCroisement > DELAI_ANTI_REBOND)) {
                        surIntersectionPrecedente = true; 
                        compteurColonnes--; 
                        dernierTempsCroisement = millis();
                        
                        Serial.print("[TEMPS REEL] Retour X franchi ! Restant : "); 
                        Serial.println(compteurColonnes);
                    }
                }
                float corr = (Kp_x * err) + (Kd_x * (err - lastErrorVert)); lastErrorVert = err;
                
                // Le robot va dans le sens inverse par rapport à l'aller
                if (directionX == 1) {
                    setRobotSpeed(-VITESSE_BASE + corr, VITESSE_BASE + corr, ((Kp_rotx/2) * err));
                }
                else {
                    setRobotSpeed(VITESSE_BASE + corr, -VITESSE_BASE + corr, (Kp_rotx * err));
                }

            } 
            else {

                // On recentre le robot
                if (directionX == 1) { 
                    setRobotSpeed(VITESSE_BASE, -VITESSE_BASE, 0.0f); 
                }
                else {setRobotSpeed(-VITESSE_BASE, VITESSE_BASE, 0.0f); 
                }
                delay(50); 

                stopmoteurs(); 
                delay(500); 

                tempsDebutMouvement = millis(); // On commence le chronomètre
                etatPhysique = RETOUR_BASE; // On passe à l'état RETOUR_BASE
            }
            break;
        }

        // Le robot recule jusqu'à la zone de départ
        case RETOUR_BASE: {

            // Même logique de suivi de ligne et de comptage qu'à l'aller, le robot doit néanmoins croiser une ligne en moins
            // pour le même trajet
            if (compteurLigne > 1) {
                float err = getErreurHoriz(); 
                if (millis() - tempsDebutMouvement > DELAI_ZONE_AVEUGLE) {
                    if (horizSurBlanc() && surIntersectionPrecedente) surIntersectionPrecedente = false; 
                    if (horizSurLigneNoire() && !surIntersectionPrecedente && (millis() - dernierTempsCroisement > DELAI_ANTI_REBOND)) {
                        surIntersectionPrecedente = true; 
                        compteurLigne--; 
                        dernierTempsCroisement = millis();
                        
                        Serial.print("[TEMPS REEL] Retour Y franchi ! Restant : "); 
                        Serial.println(compteurLigne);
                    }
                }
                float corr = (Kp_y * err) + (Kd_y * (err - lastErrorHoriz)); lastErrorHoriz = err;
                
                // La correction de rotation est divisée par deux car lorsque le robot recule 
                // la correction est plus sensible

                setRobotSpeed(-VITESSE_BASE - corr, -VITESSE_BASE + corr, -((Kp_roty/2) * err)); 
            } 
            else { 
                setRobotSpeed(VITESSE_BASE, VITESSE_BASE, 0.0f); 
                delay(50); // recentrage
                
                stopmoteurs(); 
                etatPhysique = EN_ATTENTE; 
            }
            break;
        }
    }
}







void setup() {
    // Initialise la console pour le débuggage
    Serial.begin(9600);

    // Configure les 12 broches moteurs (4 moteurs x 3 broches) en sortie
    for (int i=0; i<4; ++i) { 
        pinMode(in1Pins[i], OUTPUT); 
        pinMode(in2Pins[i], OUTPUT); 
        pinMode(pwmPins[i], OUTPUT); 
    }
    
    // Initialise les deux barrettes de capteurs QTR (Horizontale et Verticale)
    qtrHoriz.setTypeRC(); qtrHoriz.setSensorPins(sensorPinsHoriz, SensorCount);
    qtrVert.setTypeRC();  qtrVert.setSensorPins(sensorPinsVert, SensorCount);
    
    // Prépare le servo et lève le stylo pour ne pas gribouiller au démarrage
    styloServo.attach(servoPin); styloServo.write(SERVO_UP);

    // Démarre la radio ; si erreur matérielle, bloque tout ici (sécurité)
    if (!radio.begin()) while (1);
    radio.setPALevel(RF24_PA_LOW); // Puissance réduite pour tests à courte portée
    radio.setChannel(108);         // Canal 108 pour éviter les interférences WiFi
    
    // Croisement des adresses : le canal d'écriture de l'un est la lecture de l'autre
    if (jeJoueEnPremier) {
        radio.openWritingPipe(addresses[1]);
        radio.openReadingPipe(1, addresses[0]);
    } else {
        radio.openWritingPipe(addresses[0]);
        radio.openReadingPipe(1, addresses[1]);
    }
    radio.startListening();

    // Phase de calibration : la LED s'allume, on a 4 secondes pour passer 
    // les capteurs sur le noir et le blanc pour qu'ils "apprennent" le contraste
    digitalWrite(LED_BUILTIN, HIGH);
    for (int i = 0; i < 200; i++) { 
        qtrHoriz.calibrate(); 
        qtrVert.calibrate(); 
        delay(20); 
    }
    digitalWrite(LED_BUILTIN, LOW);
    
    // Affiche l'état vide du jeu au début
    draw_board(board);
}







void loop() {
    switch(etatJeu) {
        
        case INITIALISATION:
            // Au démarrage, on décide qui commence selon la variable 'jeJoueEnPremier'
            etatJeu = (jeJoueEnPremier) ? MON_TOUR_CALCUL : ATTENTE_ADVERSAIRE;
            break;

        case MON_TOUR_CALCUL:
            if (moves < 9) {
                // 1. L'IA choisit la meilleure case (k)
                int k = computerMove(board);
                board[k] = 1; moves++; // On marque la case dans le tableau
                dernierCoupJoue = k + 1; // On ajuste l'index (0-8 devient 1-9)
                
                // 2. On prépare le mouvement
                configurerDeplacementPhysique(dernierCoupJoue);
                
                // 3. On lance les moteurs et on change d'état
                etatPhysique = TRAJET_ALLER;
                etatJeu = EN_DEPLACEMENT;
            }
            break;

        case EN_DEPLACEMENT:
            // Le robot exécute son trajet (suivi de ligne, dessin, retour)
            gererDeplacementPhysique();
            
            // Si le mouvement physique est fini (le robot est revenu au repos)
            if (etatPhysique == EN_ATTENTE) {
                envoyerCoupRF(dernierCoupJoue); // On prévient l'adversaire par radio
                
                // Si la partie est finie, on coupe tout, sinon c'est au tour de l'autre
                if (win(board) != 0 || moves >= 9) etatJeu = FIN_PARTIE;
                else etatJeu = ATTENTE_ADVERSAIRE;
            }
            break;

        case ATTENTE_ADVERSAIRE:
            // On écoute la radio pour savoir où l'adversaire a joué
            if (radio.available()) {
                int coup_adverse;
                radio.read(&coup_adverse, sizeof(int));
                
                // Si le coup est valide, on met à jour la grille (-1 pour l'adversaire)
                if (coup_adverse >= 1 && coup_adverse <= 9 && board[coup_adverse-1] == 0) {
                    board[coup_adverse-1] = -1; moves++;
                    
                    // On vérifie si l'adversaire a gagné, sinon c'est à nous de calculer
                    etatJeu = (win(board) != 0 || moves >= 9) ? FIN_PARTIE : MON_TOUR_CALCUL;
                }
            }
            break;

        case FIN_PARTIE:
            // On s'assure que tout est bien arrêté à la fin du match
            stopmoteurs();
            break;
    }
}