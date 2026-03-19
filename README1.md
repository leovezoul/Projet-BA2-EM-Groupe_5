# Système Autonome de Jeu OXO : Module d'Intelligence Artificielle et Contrôle Cinématique

## 1. Description du Projet

Ce répertoire contient le code source embarqué du robot autonome conçu pour jouer au OXO sur une grille physique. Ce module constitue le "cerveau" du système : il intègre la prise de décision stratégique, la communication inter-systèmes et le contrôle bas niveau des actionneurs.
Le système est capable de recevoir l'état du jeu indiquant la case jouée par l'adversaire via un protocole radio, de calculer le coup optimal garantissant l'absence de défaite, et d'exécuter une trajectoire physique complexe pour marquer la grille.



## 2. Architecture Technique et Fonctionnalités

Le code s'articule autour de quatre axes d'ingénierie majeurs :

* **Théorie des Jeux (IA) :** Implémentation de l'algorithme récursif **Minimax**. L'arbre de décision explore l'intégralité des combinaisons possibles (espace d'états) pour déterminer le coup optimal. Le robot est mathématiquement invaincu (victoire ou match nul).
* **Cinématique Omnidirectionnelle :** Modélisation mathématique et contrôle de 4 roues Omni Wheels. Le système résout la matrice cinématique inverse pour permettre des translations latérales sans rotation du châssis.
* **Automatique (Asservissement PID) :** Régulation proportionnelle-dérivée (PD) couplée à deux matrices de capteurs infrarouges (Pololu QTR-5RC). Le système corrige en temps réel l'erreur de trajectoire sur les axes X et Y pour un suivi de ligne haute précision.
                                         Les coefficients différentiels sont mis à 0 donc la correction est uniquement proportionnelle, néanmoins la correction différentielle est prête à être implémentée en cas de nécéssité.
* **Télécommunications :** Implémentation d'une communication RF (2.4 GHz) via le module nRF24L01. Gestion de paquets binaires avec accusé de réception matériel et architecture maître-esclave alternée (Half-Duplex).



## 3. Spécifications Matérielles (Hardware Pinout)

Le système est conçu pour être déployé sur un microcontrôleur **Arduino Mega 2560** (nécessaire en raison du nombre d'entrées/sorties requis).

| Composant | Description | Broches (Pins) |
| :--- | :--- | :--- |
| **Moteurs DC (x4)** | Contrôle de puissance (PWM) | `2, 3, 4, 5` |
| **Ponts en H (L298N)** | Contrôle de direction (IN1/IN2) | `22 à 29` |
| **Capteur QTR-5RC (Y)** | Suivi de ligne longitudinal | `A0, A1, A2, A3, A4` |
| **Capteur QTR-5RC (X)** | Suivi de ligne transversal | `A5, A6, A7, A8, A9` |
| **Module NRF24L01** | Communication SPI | CE: `48`, CSN: `49`, MOSI: `51`, MISO: `50`, SCK: `52` |
| **Servomoteur** | Effecteur terminal (Stylo) | `9` |



## 4. Dépendances Logicielles

L'environnement de développement (IDE Arduino) doit intégrer les bibliothèques suivantes pour permettre la compilation :

* `SPI.h` (Standard Arduino)
* `math.h` (Standard C/C++)
* `Servo.h` (Standard Arduino)
* `QTRSensors.h` (Fournie par Pololu)
* `RF24.h` (Fournie par TMRh20)



## 5. Déploiement et Configuration

### 5.1. Paramétrage initial
Avant le téléversement (`Upload`), il est impératif de définir le rôle du robot dans le jeu en modifiant le booléen suivant (Ligne d'en-tête du fichier `.ino`) :

```cpp
// Configuration du rôle du robot (Détermine la forme dessinée et l'ordre de jeu)
bool jeJoueEnPremier = false;
