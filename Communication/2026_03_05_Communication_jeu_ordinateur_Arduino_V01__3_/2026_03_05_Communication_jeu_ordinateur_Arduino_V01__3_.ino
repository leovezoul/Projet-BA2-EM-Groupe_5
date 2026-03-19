#include <SPI.h>
#include <RF24.h>

// === CONFIGURATION RADIO ===
RF24 radio(9, 10); // Broches CE, CSN de l'Arduino branché au PC
const byte addresses[][6] = {"1Node", "2Node"};

// === CHOIX DU JOUEUR ===
// 'true' si TOI (l'humain au PC) tu joues en premier.
// 'false' si le ROBOT joue en premier.
// (Attention : ça doit être l'inverse de ce qui est configuré sur le robot !)
bool jeJoueEnPremier = false; 

// === VARIABLES GLOBALES DU JEU ===
int board[9] = {0};
int moves = 0; // Compteur des 9 tours
int historiqueCoups[9] = {0}; // Pour retenir qui a joué quoi

enum GameState { TOUR_HUMAIN, ATTENTE_ROBOT, FIN_PARTIE };
GameState etatJeu;

// ======================================
// FONCTIONS D'AFFICHAGE
// ======================================
void draw_board(int *b) { // Tableaux qui permettra de voir l'évolution du plateau sur notre pc (qd c'est nous qui jouons)
    int k = 0;
    Serial.println("----++---++----");
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Serial.print("| ");
            if (b[k] == 0) {
                Serial.print(k + 1); Serial.print(" |");
            } else if (b[k] == -1) {
                Serial.print("X |"); // L'adversaire (le robot)
            } else {
                Serial.print("O |"); // Toi (l'humain)
            }
            k++;
        }
        Serial.println("\n----++---++----");
    }
}

void afficherHistorique() {
    Serial.println("\n=== HISTORIQUE DES COUPS ===");
    for(int i = 0; i < moves; i++) {
        Serial.print("Tour "); Serial.print(i + 1);
        Serial.print(" : Case "); Serial.println(historiqueCoups[i]);
    }
    Serial.println("============================");
}

int win(const int *board) {
    unsigned wins[8][3] = {{0,1,2},{3,4,5},{6,7,8},{0,3,6},{1,4,7},{2,5,8},{0,4,8},{2,4,6}};
    for (int i = 0; i < 8; ++i) {
        if (board[wins[i][0]] != 0 &&
            board[wins[i][0]] == board[wins[i][1]] && board[wins[i][1]] == board[wins[i][2]])
            return board[wins[i][2]];
    }
    return 0;
}

// ========================================
// SETUP ARDUINO (Console PC)
// ========================================
void setup() {
    Serial.begin(9600);
    while (!Serial) { ; }

    // Initialisation Radio
    if (!radio.begin()) {
        Serial.println("NRF24 non detecte sur le PC !");
        while (1);
    }
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(108);
    radio.setRetries(15, 15);

    // Initialisation des rôles radio (inversé par rapport au robot)
    if (jeJoueEnPremier) {
        radio.openWritingPipe(addresses[1]);
        radio.openReadingPipe(1, addresses[0]);
        etatJeu = TOUR_HUMAIN; 
        Serial.println("Vous jouez en premier (O). Tapez une case (1-9) :");
    } else {
        radio.openWritingPipe(addresses[0]);
        radio.openReadingPipe(1, addresses[1]);
        etatJeu = ATTENTE_ROBOT; 
        Serial.println("Le robot joue en premier (X). En attente de son coup via radio...");
    }
    radio.startListening();

    Serial.println("\n~~~~~~~~~~~~~ CONSOLE PC - TIC-TAC-TOE ~~~~~~~~~~~~~");
    draw_board(board);
}

// ========================================
// BOUCLE PRINCIPALE (PC)
// ========================================
void loop() {
    switch(etatJeu) {

        case TOUR_HUMAIN:
            if (Serial.available() > 0) {
                char receivedChar = Serial.read();
                
                if (receivedChar >= '1' && receivedChar <= '9') {
                    int coup_humain = receivedChar - '0'; 
                    
                    if (board[coup_humain - 1] == 0) {
                        // 1. On met à jour le plateau localement
                        board[coup_humain - 1] = 1; // 1 = O (Toi)
                        historiqueCoups[moves] = coup_humain; // On sauvegarde dans l'historique
                        moves++;
                        
                        // 2. On affiche le coup
                        Serial.print("\n[VOUS] Vous avez joue la case : "); 
                        Serial.println(coup_humain);
                        draw_board(board);
                        afficherHistorique();

                        // 3. On envoie le coup au robot PAR RADIO
                        radio.stopListening();
                        radio.write(&coup_humain, sizeof(coup_humain));
                        radio.startListening();

                        // 4. On vérifie si la partie est finie
                        if (win(board) == 1) {
                            Serial.println("\n[FIN] Felicitations, vous avez gagne !");
                            etatJeu = FIN_PARTIE;
                        } else if (moves >= 9) {
                            Serial.println("\n[FIN] Match nul !");
                            etatJeu = FIN_PARTIE;
                        } else {
                            Serial.println("\n[ATTENTE] En attente du coup du robot via radio...");
                            etatJeu = ATTENTE_ROBOT; 
                        }
                    } else {
                        Serial.println("[ERREUR] Cette case est deja occupee ! Reessayez.");
                    }
                } else if (receivedChar != '\n' && receivedChar != '\r') {
                    Serial.println("[ERREUR] Entree invalide. Tapez un chiffre de 1 a 9.");
                }
            }
            break;

        case ATTENTE_ROBOT:
            if (radio.available()) {
                int coup_robot;
                radio.read(&coup_robot, sizeof(coup_robot));
                
                if (coup_robot >= 1 && coup_robot <= 9 && board[coup_robot - 1] == 0) {
                    // 1. On met à jour le plateau
                    board[coup_robot - 1] = -1; // -1 = X (Le Robot)
                    historiqueCoups[moves] = coup_robot; // On ajoute à l'historique
                    moves++;

                    // 2. On affiche l'action du robot
                    Serial.print("\n[ROBOT] Le robot a joue la case : "); 
                    Serial.println(coup_robot);
                    draw_board(board);
                    afficherHistorique();

                    // 3. On vérifie la fin de partie
                    if (win(board) == -1) {
                        Serial.println("\n[FIN] Le robot a gagne !");
                        etatJeu = FIN_PARTIE;
                    } else if (moves >= 9) {
                        Serial.println("\n[FIN] Match nul !");
                        etatJeu = FIN_PARTIE;
                    } else {
                        Serial.println("\n[A VOUS] Tapez votre coup (1-9) :");
                        etatJeu = TOUR_HUMAIN; 
                    }
                } else {
                    Serial.println("[ERREUR RADIO] Coup recu invalide !");
                }
            }
            break;

        case FIN_PARTIE:
            // Terminé. 
            break;
    }
}
