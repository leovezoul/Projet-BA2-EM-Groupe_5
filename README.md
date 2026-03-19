# Projet-BA2-EM-Groupe_5

Ce github présente les codes d'un système robotique omnidirectionnel conçu pour jouer physiquement au OXO de manière totalement autonome ou alors en jouant les cases demandées. 

Ce projet multidisciplinaire intègre des concepts avancés d'**Intelligence Artificielle** (Théorie des jeux), de **Cinématique** (déplacement avec des Omni wheels), de **suivi de ligne** (Asservissement PID), de **Télécommunications** (Module radio nRF24L01) et de **Simulation**.



---

## 📑 Sommaire du github (Architecture du Projet)

Ce répertoire est divisé en 4 parties distinctes. Chacune possède son propre sous-répertoire contenant le code source et une documentation technique détaillée.

### 1. [Mode Autonome avec IA Minimax](./1_Robot_IA_Minimax)
Le "cerveau" complet du robot. Dans ce mode, le système calcule le coup mathématiquement parfait grâce à l'algorithme récursif **Minimax**. Il écoute le coup de l'adversaire par radio, détermine sa réponse, et exécute physiquement la trajectoire pour aller marquer la grille avec son un stylo.
* **Technologies :** C++ (Arduino), Arbre de décision Minimax, Régulation PID.



###  2. [Mode Esclave avec instructions](./2_Robot_commandé)
Une version allégée du micrologiciel embarqué. L'IA est désactivée pour transformer le robot en une unité d'exécution pure. Il attend la réception d'une coordonnée (1 à 9) via radio et exécute la trajectoire physique correspondante. Idéal pour les tests d'étalonnage cinématique et lorsqu'on le robot doit aller sur une case spécifique
* **Technologies :** C++ (Arduino), Machine à états finis, Matrices de capteurs optiques.

### 3. [Code de communication du PC](./3_communication_PC)
Le code de la station de contrôle au sol. Ce micrologiciel transforme un Arduino relié en USB à un ordinateur en un pont de communication (Bridge UART/RF). Il permet à un humain d'envoyer des commandes chiffrées au robot depuis le Moniteur Série.
* **Technologies :** C++ (Arduino), Protocole SPI, Transceiver nRF24L01 (Auto-ACK).

### 4. [Simulation Environnementale (CoppeliaSim)](./4_Simulation)
La version numérique du projet. Un environnement de simulation physique permettant de valider les algorithmes et la logique de contrôle de la machine à états avant le déploiement sur le matériel réel.
* **Technologies :** Lua, CoppeliaSim / V-REP, Modélisation cinématique (Configuration en croix '+'), Capteurs virtuels de vision.



---

## ⚙️ Spécifications Matérielles Générales (Hardware)

Le système physique repose sur l'architecture matérielle suivante :

* **Unité de Traitement :** Arduino Mega 2560 (Robot) & Arduino Uno/Nano (Communication PC)
* **Mobilité :** Châssis à 4 roues Mecanum en configuration X drive motorisées indépendamment pour un déplacement holonome (translations directes sans rotation).
* **Perception (Odométrie optique) :** 2x Barrettes de capteurs infrarouges Pololu QTR-5RC (Lecture de contraste pour suivi de ligne X et Y).
* **Communication :** Modules Transceivers RF nRF24L01+ (Communication Half-Duplex sur la bande 2.4 GHz).
* **Actionneur :** Servomoteur standard (Gestion de l'axe Z pour l'écriture).



---

## Installation et Dépendances

### Prérequis Logiciels
* **Arduino IDE** (v1.8.x ou supérieure) pour la compilation du code C++.
* **CoppeliaSim** (Anciennement V-REP) pour l'exécution du script Lua de simulation.

### Bibliothèques Arduino Requises
Pour compiler les codes des dossiers `1`, `2` et `3`, installez les dépendances suivantes via le Gestionnaire de Bibliothèques de l'IDE Arduino :
* `QTRSensors` (par Pololu) - Gestion des matrices infrarouges.
* `RF24` (par TMRh20) - Gestion du protocole radio SPI.
* `Servo` (Standard) - Contrôle de l'effecteur.

### Démarrage Rapide
1. Clonez ce répertoire sur votre machine locale :
   ```bash
   git clone [https://github.com/VOTRE_NOM_UTILISATEUR/TicTacToe-Mecanum-Robot.git](https://github.com/VOTRE_NOM_UTILISATEUR/TicTacToe-Mecanum-Robot.git)
