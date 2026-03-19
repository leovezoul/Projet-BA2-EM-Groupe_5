/*Code permettant d'envoyer un int au robot par radio
afin d'informer le robot où l'adversaire à joué */



#include <SPI.h>
#include <RF24.h>



// Initialisation du module radio sur les broches CE (9) et CSN (10)
RF24 radio(9, 10);

// Déclaration des "tuyaux" (pipes) de communication.
// Identiques à ceux du robot pour qu'ils parlent la même langue.
const byte addresses[][6] = {"1Node", "2Node"};


void setup() {
    // 1. INITIALISATION DE LA CONSOLE
    // Permet de lire les messages sur l'ordinateur et d'envoyer nos frappes clavier
    Serial.begin(9600);
    
    // 2. VÉRIFICATION MATÉRIELLE RADIO
    // Si la puce NRF24 n'est pas bien branchée, on bloque le programme ici (while infinie)
    if (!radio.begin()) {
        Serial.println("NRF24 Manette non détecté!");
        while (1);
    }
    
    // 3. PARAMÉTRAGE RADIO (Doit être strictement identique au robot)
    radio.setPALevel(RF24_PA_LOW);   // Puissance basse (idéal pour des tests sur un bureau)
    radio.setDataRate(RF24_1MBPS);   // Vitesse de transmission standard
    radio.setChannel(108);           // Canal 108 (hors des fréquences WiFi classiques)
    
    // 4. CROISEMENT DES ADRESSES
    // La manette écrit sur l'adresse 0 et écoute sur la 1.
    // (Attention : le robot devra donc obligatoirement écouter sur la 0 et écrire sur la 1).
    radio.openWritingPipe(addresses[0]); 
    radio.openReadingPipe(1, addresses[1]);
    
    // On se met en mode écoute par défaut
    radio.startListening();

    // 5. MESSAGE D'ACCUEIL
    Serial.println("=== MANETTE PC PRÊTE ===");
    Serial.println("Tape la case (1-9) pour l'envoyer au robot.");
}


void loop() {

    // Vérifie si l'utilisateur a tapé quelque chose dans le Moniteur Série
    if (Serial.available() > 0) {
        
        // On lit le premier caractère tapé
        char input = Serial.read();
        
        // Sécurité : on s'assure que c'est bien un chiffre entre 1 et 9
        if (input >= '1' && input <= '9') {
            
            // CONVERSION ASCII -> ENTIER
            // Soustraire '0' (qui vaut 48 en ASCII) transforme le texte '5' en vrai chiffre 5
            int coup = input - '0';
            
            // SÉQUENCE D'ENVOI RADIO
            radio.stopListening(); // On coupe l'écoute pour libérer l'antenne
            
            // La fonction write() renvoie 'true' si le robot a bien reçu et accusé réception (Auto-ACK)
            bool ok = radio.write(&coup, sizeof(coup));
            
            radio.startListening(); // On se remet immédiatement sur écoute
            
            // AFFICHAGE DU RÉSULTAT DE TRANSMISSION
            if (ok) {
                Serial.print("[PC] Coup envoyé au robot : "); Serial.println(coup);
            } else {
                Serial.println("[ERREUR] Impossible d'envoyer (Robot éteint ou hors de portée ?)");
            }
        }
        
        // NETTOYAGE DU BUFFER SÉRIE (TRÈS IMPORTANT)
        // Vide tous les caractères invisibles restants (comme 'Entrée' -> \n ou \r)
        // Cela évite que la boucle relance une commande vide au tour suivant.
        while(Serial.available() > 0) Serial.read();
    }





    // Vérifie si un message radio est arrivé dans l'antenne
    if (radio.available()) {
        int coup_robot;
        
        // On lit le paquet de données (la taille d'un 'int', soit 2 octets)
        radio.read(&coup_robot, sizeof(coup_robot));
        
        // On affiche le message reçu
        Serial.print("\n[INFO] Le robot a répondu : "); Serial.println(coup_robot);
    }
}