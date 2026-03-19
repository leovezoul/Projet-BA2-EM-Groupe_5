# Système de Navigation Piloté : Module de Déplacement sur Grille par Commande Radio

## 1. Description du Projet

Ce répertoire contient le code source permettant au robot d'agir comme une unité d'exécution physique télécommandée. Contrairement au module avec l'intelligence décisionnelle (Minimax), il attend la réception d'un index de case (1 à 9) via une liaison RF pour ensuite se rendre sur la case recue. 

Ce code permet de tester le déplacement du robot en lui donnant l'ordre d'aller sur la case souhaitée. Cette fonctionnalité est également utile dans le cadre de l'évaluation du robot nécéssitant que celui-ci aille à différentes cases prédéfinies. 

## 2. Architecture et Logique de Fonctionnement

Le programme repose sur une interaction constante entre la réception de données et la machine à états physique :

* **Réception de Consigne :** Le système surveille le tampon de réception du module nRF24L01. Lorsqu'une donnée entière (int) est détectée, elle est traitée comme la coordonnée cible (`cibleActuelle`).
* **Séquençage du Mouvement :** Une fois la cible définie, le robot active sa machine à états (`EtatPhysique`) pour enchaîner les phases de translation longitudinale, translation transversale, marquage au sol, et retour au point d'origine.
* **Interprétation du Rôle :** Le booléen `jeJoueEnPremier` définit la forme dessinée (Croix ou Rond) et adapte les paramètres de comptage de lignes selon la géométrie du point de départ.

## 3. Schéma de Câblage et Affectation des Broches (Pinout)

L'affectation des entrées/sorties sur l'**Arduino Mega 2560** est identique au module autonome pour garantir une compatibilité matérielle totale.

| Bloc Fonctionnel | Pins Arduino |
| :--- | :--- |
| **Moteurs (PWM)** | `2, 3, 4, 5` |
| **Moteurs (Direction)** | `22, 23, 24, 25, 26, 27, 28, 29` |
| **Capteurs QTR (Horiz)** | `A0, A1, A2, A3, A4` |
| **Capteurs QTR (Vert)** | `A5, A6, A7, A8, A9` |
| **nRF24L01 (SPI)** | CE: `48`, CSN: `49` |
| **Servomoteur** | `9` |

## 4. Instructions de Mise en Œuvre

### 4.1. Configuration du Logiciel (Pré-téléversement)
Avant d'envoyer le code vers la carte, vérifiez la ligne d'en-tête suivante :
```cpp
bool jeJoueEnPremier = false;
