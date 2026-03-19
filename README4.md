# Station de Contrôle PC : Module d'Émission de Consignes via Interface Série

## 1. Description du Projet

Ce répertoire contient le micrologiciel permettant d'utiliser un microcontrôleur Arduino comme passerelle de communication entre un ordinateur et le robot. Ce module fait office de "Manette" : il traduit les entrées clavier saisies dans le moniteur série en paquets de données radiofréquences (RF) intelligibles pour le récepteur du robot.

Il permet un pilotage manuel direct, essentiel pour valider la chaîne cinématique et la précision du suivi de ligne sans intervention de l'IA embarquée.

## 2. Architecture de Communication

Le système fonctionne sur un principe de pont de données (Bridge) :

* **Interface Série (UART) :** Le PC envoie un caractère ASCII (ex: '5') à l'Arduino via le câble USB.
* **Traitement de Données :** L'Arduino convertit ce caractère en un entier mathématique (`int`).
* **Liaison Radio (RF) :** L'entier est encapsulé et transmis sur la bande 2.4 GHz via le protocole nRF24L01.
* **Accusé de Réception (ACK) :** Le module attend une confirmation matérielle du robot. Si le robot reçoit l'ordre, la manette confirme le succès sur l'écran du PC.

## 3. Configuration Matérielle (Hardware)

Ce code est optimisé pour un **Arduino Uno** ou **Nano** branché en USB au PC.

| Composant | Broche Arduino | Rôle |
| :--- | :--- | :--- |
| **nRF24L01 CE** | `9` | Activation Chip |
| **nRF24L01 CSN** | `10` | Sélection SPI |
| **nRF24L01 MOSI** | `11` | Données Sortantes |
| **nRF24L01 MISO** | `12` | Données Entrantes |
| **nRF24L01 SCK** | `13` | Horloge SPI |

## 4. Protocole d'Utilisation

### 4.1. Configuration des "Pipes" (Adresses)
Pour établir la liaison, les adresses doivent être "croisées" avec celles du robot. Par défaut, ce code est configuré comme suit :
* **Écriture (TX) :** `addresses[0]` ("1Node")
* **Lecture (RX) :** `addresses[1]` ("2Node")

### 4.2. Manipulation via le Moniteur Série
1.  Connectez l'Arduino au PC et téléversez le code.
2.  Ouvrez le **Moniteur Série** (Outils > Moniteur Série).
3.  Réglez la vitesse sur **9600 bauds**.
4.  Réglez le paramètre de fin de ligne sur **"Pas de fin de ligne"** (ou "No line ending").
5.  Saisissez un chiffre entre **1 et 9** et appuyez sur **Entrée**.

## 5. Sécurité et Fiabilité du Code

* **Nettoyage du Tampon (Buffer) :** Le code intègre une routine `while(Serial.available() > 0) Serial.read();` qui vide les caractères résiduels après chaque envoi. Cela garantit qu'un appui accidentel ou un retour chariot ne génère pas d'ordres fantômes.
* **Vérification de Transmission :** La fonction `radio.write()` est utilisée de manière booléenne. En cas de non-réponse du robot (batterie vide, obstacle, interférence), la manette affiche immédiatement une alerte d'erreur sur le moniteur série.
