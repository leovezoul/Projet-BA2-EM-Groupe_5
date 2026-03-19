# Station de Contrôle PC : Module d'Émission de Consignes via Interface Série

## 1. Description du Projet

Ce répertoire contient le micrologiciel permettant d'utiliser un microcontrôleur Arduino comme passerelle de communication entre un ordinateur et le robot. Ce module fait office de **"Manette"** : il traduit les entrées clavier saisies dans le moniteur série en paquets de données radiofréquences (RF) intelligibles pour le récepteur du robot.

Ce module permet l'émission d'une **consigne numérique comprise entre 1 et 9** vers le récepteur du robot. L'interprétation de cet entier par le système embarqué dépend du micrologiciel préalablement téléversé sur le robot :
* **Mode IA (Minimax) :** L'entier est traité comme la coordonnée du coup joué par l'adversaire humain.
* **Mode Piloté :** L'entier est traité comme une commande directe de positionnement vers la case cible.

## 2. Architecture de Communication

Le système fonctionne sur un principe de pont de données (**Bridge**) :

* **Interface Série (UART) :** Le PC envoie un caractère **ASCII** (ex: '5') à l'Arduino via le câble USB. 
* **Traitement de Données :** L'Arduino convertit ce symbole ASCII en une valeur mathématique utilisable (`int`) en soustrayant le caractère '0' (Code ASCII 48).
* **Liaison Radio (RF) :** L'entier est encapsulé dans un paquet 2.4 GHz via le module nRF24L01.
* **Accusé de Réception (Auto-ACK) :** Le module attend une confirmation matérielle (Handshake) du robot. Si le robot reçoit l'ordre, la manette confirme le succès sur l'écran du PC.



## 3. Configuration Matérielle (Hardware)

Ce code est optimisé pour un **Arduino Uno** ou **Nano** branché en USB au PC.

| Composant | Broche Arduino | Rôle |
| :--- | :--- | :--- |
| **nRF24L01 CE** | `9` | Activation du circuit radio |
| **nRF24L01 CSN** | `10` | Sélection du bus SPI |
| **nRF24L01 MOSI** | `11` | Master Out Slave In (Données Sortantes) |
| **nRF24L01 MISO** | `12` | Master In Slave Out (Données Entrantes) |
| **nRF24L01 SCK** | `13` | Horloge de synchronisation SPI |

## 4. Protocole d'Utilisation et Compatibilité

### 4.1. Configuration impérative du Robot
Pour que la liaison soit établie, le code du **Robot** doit impérativement posséder une configuration "miroir" de celle de la manette :

1.  **Canal identique :** Le robot doit être sur le canal 108 (`radio.setChannel(108);`).
2.  **Croisement des Pipes (Adresses) :** La manette écrit sur un tuyau et écoute sur l'autre. Le robot doit inverser ce schéma.
    * **Côté Manette PC :** Écrit sur `addresses[0]`, Écoute sur `addresses[1]`.
    * **Côté Robot :** Doit Écrire sur `addresses[1]`, Écouter sur `addresses[0]`.
3.  **Vitesse de données :** Les deux modules doivent être réglés sur 1MBPS (`radio.setDataRate(RF24_1MBPS);`).



### 4.2. Manipulation via le Moniteur Série
1.  Connectez l'Arduino au PC et téléversez le code.
2.  Ouvrez le **Moniteur Série** (Outils > Moniteur Série).
3.  Réglez la vitesse sur **9600 bauds**.
4.  Réglez le paramètre de fin de ligne sur **"Pas de fin de ligne"** (No line ending).
5.  Saisissez un chiffre entre **1 et 9** et appuyez sur **Entrée**.

## 5. Sécurité et Fiabilité du Code

* **Nettoyage du Tampon (Buffer) :** Le code intègre une routine `while(Serial.available() > 0) Serial.read();` qui vide les caractères résiduels (comme les retours à la ligne invisibles `\n` ou `\r`). Cela garantit qu'aucune commande "fantôme" n'est envoyée par erreur.
* **Vérification de Transmission :** La fonction `radio.write()` est exploitée comme un test booléen. En utilisant les 15 tentatives de réémission automatiques configurées (`setRetries`), elle ne renvoie une erreur que si la liaison physique est réellement rompue (distance excessive, obstacle majeur ou robot hors tension).
