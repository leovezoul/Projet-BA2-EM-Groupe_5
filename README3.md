# Documentation Technique - Simulateur de Navigation Autonome (Tic-Tac-Toe)

Ce module Lua implémente la logique de contrôle et de navigation d'un robot omnidirectionnel (configuration en croix '+') au sein d'un environnement de simulation physique (type CoppeliaSim / V-REP). Le système permet au robot de se déplacer de manière autonome sur une grille cartésienne pour atteindre une case cible (de 1 à 9), en s'appuyant sur un réseau de capteurs optiques et une régulation PID.

## 1. Architecture Matérielle Virtuelle

Le script requiert la présence des actionneurs et capteurs suivants dans la hiérarchie de la scène (les handles sont récupérés dynamiquement au lancement) :

### 1.1. Actionneurs (Joints)
La cinématique omnidirectionnelle est assurée par 4 roues motorisées indépendantes :
* `joint_1` : Moteur arrière (Axe X)
* `joint_2` : Moteur avant (Axe X)
* `joint_3` : Moteur gauche (Axe Y)
* `joint_4` : Moteur droit (Axe Y)

### 1.2. Capteurs Optiques (Vision Sensors)
Une matrice de 9 capteurs infrarouges (`qtr1` à `qtr9`) est utilisée pour le suivi de ligne et la détection d'intersections :
* **Détection de l'erreur latérale (Axe Y) :** Capteurs `qtr1` à `qtr5` (alignement sur l'axe X).
* **Détection de l'erreur longitudinale (Axe X) :** Capteurs `qtr6` à `qtr9` + `qtr3` (alignement sur l'axe Y).

---

## 2. Modèle Mathématique de Routage

Afin de convertir l'index de la case cible $c \in [1, 9]$ en coordonnées de grille spatiales exploitables par la machine à états, le script intègre un modèle de routage dynamique.

### 2.1. Calcul de l'objectif longitudinal (Rangée)
Le nombre de lignes horizontales à franchir (Axe Y) est déterminé par :
$Y = \lfloor \frac{c - 1}{3} \rfloor + 3$
*(L'offset de +3 compense la distance initiale entre le point d'apparition du robot et la première ligne de la zone de jeu).*

### 2.2. Calcul de l'objectif latéral (Colonne)
La direction et la distance de la translation latérale (Axe X) sont calculées par l'opération modulo :
$X = (c - 1) \pmod{3} + 1$
* Si $X = 1$ : Translation à gauche (Direction = -1).
* Si $X = 2$ : Position centrale (Direction = 0, aucune translation requise).
* Si $X = 3$ : Translation à droite (Direction = 1).

---

## 3. Logique de Contrôle (Machine à États)

Le comportement du robot est régi par une machine à états finis (`etatActuel`), garantissant une séparation stricte des axes de déplacement pour minimiser l'erreur odométrique.

* **État 1 (Avancée Longitudinale) :** Le robot se déplace sur l'axe Y. Un régulateur de type Proportionnel (P) agit sur l'axe X pour maintenir le cap. Le système compte les intersections horizontales jusqu'à atteindre la rangée cible.
* **État 2 (Translation Latérale / Glissade) :** Déplacement exclusif sur l'axe X. Le régulateur P bascule sur l'axe Y pour garantir le maintien sur la rangée en cours. Le système compte les intersections verticales.
* **État 3 (Arrêt et Positionnement) :** Les consignes de vitesse de l'ensemble des moteurs sont ramenées à zéro. La cible est atteinte.

---

## 4. Déploiement et Utilisation

1. **Définition de la consigne :** Dans la fonction d'initialisation `sysCall_init()`, assignez la valeur désirée à la variable `targetCase`.
   ```lua
   targetCase = 5  -- Valeur entière comprise entre 1 et 9
