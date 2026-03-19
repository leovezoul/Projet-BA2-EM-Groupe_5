-- ========================================================================
-- SIMULATEUR DE NAVIGATION - ROBOT OMNIDIRECTIONNEL (OXO)
-- ========================================================================

function sysCall_init()
    -- Cette fonction est appelée une seule fois au lancement de la simulation.
    
    -- 1. CONNEXION AUX MOTEURS (Handles)
    -- On relie les variables du script aux objets 3D de la scène (Configuration en croix '+')
    j1 = sim.getObjectHandle('joint_1') -- Moteur Arrière (Gère l'axe X)
    j2 = sim.getObjectHandle('joint_2') -- Moteur Avant (Gère l'axe X)
    j3 = sim.getObjectHandle('joint_3') -- Moteur Gauche (Gère l'axe Y)
    j4 = sim.getObjectHandle('joint_4') -- Moteur Droit (Gère l'axe Y)
    
    corpsRobot = sim.getObjectHandle('structure') 
    
    -- 2. CONNEXION AUX CAPTEURS DE LIGNE (QTR 1 à 9)
    -- On crée un tableau (array) contenant les 9 capteurs infrarouges
    qtr = {}
    for i=1,9 do qtr[i] = sim.getObjectHandle('qtr'..i) end

    -- 3. PARAMÈTRES DE RÉGULATION PID ET VISION
    kp = 5.0            -- Gain Proportionnel : force de correction si le robot dévie de la ligne
    seuilNoir = 0.15    -- Seuil de luminosité en dessous duquel on considère voir une ligne noire

    -- 4. PARAMÈTRES DE MISSION
    targetCase = 9      -- <--- C'EST ICI QUE L'ON CHOISIT LA CASE CIBLE (1 à 9)
    
    -- 5. VARIABLES D'ÉTAT DE LA NAVIGATION
    etatActuel = 1      -- Machine à états : 1 = Avance (Axe Y), 2 = Glisse (Axe X), 3 = Stop
    vitesse = 3.0       -- Vitesse de base des moteurs
    
    pidActif = false        -- Désactivé au départ pour franchir la première ligne
    limiteFranchie = false  -- Permet d'ignorer la ligne de départ de la zone du robot
    compteurH, compteurV = 0, 0 -- Compteurs d'intersections (Horizontales et Verticales)
    objH, objV = 0, 0       -- Objectifs à atteindre (calculés plus bas)
    dirV = 0                -- Direction de la glissade (-1: Gauche, 0: Centre, 1: Droite)
    
    phaseH = 0 
    lastStateH, lastStateV = false, false -- Mémoire de l'état précédent (pour ne compter qu'une fois par ligne)
    dansLeBlanc = false     -- Vérifie que le robot a bien quitté une ligne avant de chercher la suivante

    -- Calcul automatique du chemin selon la case choisie
    configurerCible(targetCase)
end


-- ========================================================================
-- FONCTIONS OUTILS (VISION ET MATHÉMATIQUES)
-- ========================================================================

-- Vérifie si un capteur spécifique voit du noir
function voitNoir(idx)
    -- sim.readVisionSensor renvoie les données du capteur. L'index 11 contient l'intensité moyenne.
    local res, p = sim.readVisionSensor(qtr[idx])
    return (p and p[11] < seuilNoir)
end

-- Calcule l'erreur de positionnement du robot par rapport au centre de la ligne
function getError(sensorIndices)
    local sum, count = 0, 0
    -- On fait une moyenne pondérée des capteurs qui voient du noir
    for i, idx in ipairs(sensorIndices) do
        if voitNoir(idx) then
            sum = sum + ((i-1) * 1000)
            count = count + 1
        end
    end
    -- Si des capteurs voient la ligne, on renvoie une erreur (négative si trop à gauche, positive si trop à droite)
    if count > 0 then return (sum / count - 2000) / 1000 end
    return 0 -- Si aucun capteur ne voit la ligne, l'erreur est nulle
end

-- Convertit un numéro de case (1-9) en coordonnées de grille (Rangée / Colonne)
function configurerCible(c)
    -- Calcul de la rangée (Axe Y) : On divise par 3. L'offset +3 correspond à la distance de départ.
    local row = math.floor((c - 1) / 3) + 3
    objH = row 
    
    -- Calcul de la colonne (Axe X) avec un modulo 3.
    local col = (c - 1) % 3 + 1
    if col == 1 then dirV = -1 ; objV = 1     -- Glissade à gauche d'une case
    elseif col == 3 then dirV = 1 ; objV = 1  -- Glissade à droite d'une case
    else dirV = 0 ; objV = 0 end              -- Reste au centre (pas de glissade)
    
    print(string.format("[MISSION] Case: %d | Rangée: %d | Colonne: %d", c, row-1, col))
end


-- ========================================================================
-- BOUCLE DE CONTRÔLE PRINCIPALE (Exécutée à chaque pas de simulation)
-- ========================================================================

function sysCall_actuation()
    -- Calcul des erreurs de trajectoire sur les deux axes
    local errX = getError({1, 2, 3, 4, 5}) -- Capteurs alignés en X (détectent l'erreur en Y)
    local errY = getError({6, 7, 3, 8, 9}) -- Capteurs alignés en Y (détectent l'erreur en X)
    
    local vx, vy = 0, 0 -- Vitesses de consigne pour les axes X et Y

    -- --------------------------------------------------------------------
    -- ETAT 1 : AVANCEMENT LONGITUDINAL (VERS L'AVANT)
    -- --------------------------------------------------------------------
    if etatActuel == 1 then
        vy = vitesse                                -- On avance à vitesse constante
        vx = pidActif and (kp * errX) or 0          -- Le PID corrige la trajectoire sur l'axe X

        -- Détection des intersections horizontales
        local surLigneH = voitNoir(6) or voitNoir(9)
        
        if phaseH == 0 then 
            if not voitNoir(3) then phaseH = 1 end -- On attend que le capteur central sorte de la zone de départ
        elseif phaseH == 1 then
            -- Si on détecte une nouvelle ligne
            if surLigneH and not lastStateH then
                if not limiteFranchie then
                    limiteFranchie = true -- On ignore la première ligne (limite du terrain)
                    print("[NAV] Ligne 1 (Limite) ignorée")
                else
                    compteurH = compteurH + 1 -- On compte la ligne franchie
                    pidActif = true           -- On active la correction de trajectoire
                    print(string.format("[NAV] Ligne %d/%d : PID ACTIF", compteurH, objH-1))
                    
                    -- Si on a atteint la bonne rangée
                    if compteurH >= objH-1 then
                        if dirV == 0 then 
                            etatActuel = 3 -- Arrêt direct si on doit jouer sur la colonne centrale
                        else 
                            etatActuel = 2 -- Passage en mode glissade si on doit jouer sur les côtés
                        end
                    end
                end
            end
        end
        lastStateH = surLigneH -- Mise à jour de la mémoire d'état

    -- --------------------------------------------------------------------
    -- ETAT 2 : TRANSLATION LATÉRALE (GLISSADE GAUCHE / DROITE)
    -- --------------------------------------------------------------------
    elseif etatActuel == 2 then
        vx = dirV * vitesse            -- On glisse vers la gauche ou la droite selon 'dirV'
        vy = -1 * (kp * errY)          -- Le PID corrige l'erreur sur l'axe Y pour rester sur la rangée

        -- Détection des intersections verticales
        local surLigneV = voitNoir(1) or voitNoir(5)
        
        -- On s'assure d'avoir quitté la ligne précédente avant de compter la suivante
        if not (voitNoir(1) or voitNoir(5)) then dansLeBlanc = true end
        
        if dansLeBlanc and surLigneV and not lastStateV then
            compteurV = compteurV + 1 -- On a franchi une intersection verticale
            if compteurV >= objV then 
                print("[NAV] Case atteinte : ARRET")
                etatActuel = 3 -- Objectif atteint, on passe à l'arrêt
            end
        end
        lastStateV = surLigneV -- Mise à jour de la mémoire d'état

    -- --------------------------------------------------------------------
    -- ETAT 3 : ARRÊT FINAL
    -- --------------------------------------------------------------------
    elseif etatActuel == 3 then
        vx, vy = 0, 0 -- On coupe les consignes de vitesse
    end
    
    -- --------------------------------------------------------------------
    -- APPLICATION DES VITESSES AUX MOTEURS (Cinématique Omnidirectionnelle)
    -- --------------------------------------------------------------------
    sim.setJointTargetVelocity(j3, vy)   -- Moteur Gauche
    sim.setJointTargetVelocity(j4, -vy)  -- Moteur Droit (inversé car monté en miroir)
    sim.setJointTargetVelocity(j2, vx)   -- Moteur Avant
    sim.setJointTargetVelocity(j1, -vx)  -- Moteur Arrière (inversé car monté en miroir)
end

-- ========================================================================
-- NETTOYAGE EN FIN DE SIMULATION
-- ========================================================================
function sysCall_cleanup()
    -- Par sécurité, on arrête tous les moteurs quand on stoppe la simulation
    sim.setJointTargetVelocity(j1, 0)
    sim.setJointTargetVelocity(j2, 0)
    sim.setJointTargetVelocity(j3, 0)
    sim.setJointTargetVelocity(j4, 0)
end
