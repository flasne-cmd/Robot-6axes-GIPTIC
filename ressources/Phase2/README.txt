═══════════════════════════════════════════════════════════════════════════════
  PHASE 2 - PRISE EN MAIN PROGRAMMATION
  Bras robotique 6 axes Dynamixel AX-12
  GIPTIC - Formation robotique pédagogique 2026
═══════════════════════════════════════════════════════════════════════════════

📦 CONTENU DU PACK PÉDAGOGIQUE
═══════════════════════════════════════════════════════════════════════════════

1. Phase2_Guide_Formateur.docx (25 pages)
   ├─ Instructions détaillées pour l'enseignant
   ├─ Déroulé pédagogique minute par minute
   ├─ Solutions aux exercices
   ├─ Gestion des difficultés courantes
   └─ Grilles d'évaluation

2. Phase2_Support_TP.docx (15 pages)
   ├─ Document de travail pour les participants
   ├─ Exercices guidés pas à pas
   ├─ Zones de prise de notes
   └─ QCM d'auto-évaluation

3. robot_control.py (1064 lignes)
   ├─ Code Python complet et commenté
   ├─ Interface graphique Tkinter
   ├─ Cinématique directe et inverse
   ├─ Génération de trajectoires
   └─ Mode simulation intégré


🎯 OBJECTIFS DE LA PHASE 2
═══════════════════════════════════════════════════════════════════════════════

À l'issue de cette séance de 1h15, les participants seront capables de :

  ✓ Maîtriser la bibliothèque de contrôle Python (dynamixel-sdk, pyserial)
  ✓ Comprendre la cinématique directe et inverse
  ✓ Piloter le bras en mode manuel puis séquentiel
  ✓ Enregistrer et rejouer des séquences (teach-in)
  ✓ Générer des trajectoires interpolées
  ✓ Déboguer les problèmes de communication série


📋 PRÉREQUIS
═══════════════════════════════════════════════════════════════════════════════

MATÉRIEL :
  □ PC avec Python 3.8+ installé
  □ Bras robot 6 axes assemblé
  □ 6 servomoteurs Dynamixel AX-12 (IDs 1-6)
  □ Contrôleur USB2AX ou U2D2
  □ Alimentation 12V 5A minimum
  □ Câble USB

LOGICIELS :
  □ Python 3.8 ou supérieur
  □ Bibliothèques : dynamixel-sdk, pyserial, numpy, matplotlib, tkinter

CONNAISSANCES :
  □ Python niveau intermédiaire (fonctions, classes, boucles)
  □ Notions de trigonométrie (cosinus, sinus)
  □ Bases de la robotique (articulations, degrés de liberté)


⚙️ INSTALLATION RAPIDE
═══════════════════════════════════════════════════════════════════════════════

1. INSTALLER LES BIBLIOTHÈQUES PYTHON
   
   Windows :
   > pip install dynamixel-sdk pyserial numpy matplotlib

   Linux/Mac :
   $ pip3 install dynamixel-sdk pyserial numpy matplotlib
   $ sudo apt-get install python3-tk  # Si nécessaire sous Linux

2. VÉRIFIER L'INSTALLATION
   
   > python -c "import dynamixel_sdk; import numpy; import matplotlib; print('✅ OK')"
   
   Résultat attendu : ✅ OK

3. CONFIGURER LE PORT SÉRIE
   
   Ouvrir robot_control.py et modifier la ligne 49 :
   
   Windows :     DEVICENAME = "COM3"      # Vérifier dans Gestionnaire de périphériques
   Linux :       DEVICENAME = "/dev/ttyUSB0"
   Mac :         DEVICENAME = "/dev/tty.usbserial-*"

4. LANCER LE PROGRAMME
   
   > python robot_control.py
   
   L'interface graphique s'ouvre. Si le robot n'est pas connecté, le mode 
   simulation est activé automatiquement.


🚀 UTILISATION
═══════════════════════════════════════════════════════════════════════════════

MODE MANUEL :
  1. Déplacer les sliders pour contrôler chaque servomoteur
  2. Observer la visualisation 3D en temps réel
  3. Lire les coordonnées cartésiennes (X, Y, Z) de l'effecteur

TEACH-IN (ENREGISTREMENT DE SÉQUENCES) :
  1. Positionner le robot avec les sliders
  2. Cliquer sur "📝 Enregistrer position"
  3. Répéter pour créer une séquence de 4-5 positions
  4. Cliquer sur "▶️  Rejouer séquence"
  5. Observer l'exécution automatique

CINÉMATIQUE INVERSE :
  1. Définir une position cible (X, Y, Z) en cm
  2. Le programme calcule les angles nécessaires
  3. Les servos se déplacent automatiquement
  4. Tester plusieurs positions pour explorer le workspace

GÉNÉRATION DE TRAJECTOIRES :
  1. Définir un point de départ (X1, Y1, Z1)
  2. Définir un point d'arrivée (X2, Y2, Z2)
  3. Choisir le nombre de points intermédiaires (steps)
  4. Lancer l'exécution de la trajectoire
  5. Observer le mouvement fluide


📚 STRUCTURE DU CODE PYTHON
═══════════════════════════════════════════════════════════════════════════════

robot_control.py
├─ Point3D                   → Classe pour points 3D
├─ RobotController           → Contrôle des servos et cinématique
│  ├─ __init__()             → Initialisation et connexion
│  ├─ move_servo()           → Commande position
│  ├─ read_position()        → Lecture capteurs
│  ├─ forward_kinematics()   → Calcul position depuis angles
│  ├─ inverse_kinematics()   → Calcul angles depuis position
│  ├─ generate_trajectory()  → Génération trajectoires
│  └─ execute_trajectory()   → Exécution séquentielle
│
└─ RobotGUI                  → Interface graphique Tkinter
   ├─ create_sliders()       → Création des contrôles
   ├─ create_3d_plot()       → Visualisation matplotlib
   ├─ update_display()       → Rafraîchissement
   └─ callbacks              → Gestion des événements


🔧 RÉSOLUTION DES PROBLÈMES
═══════════════════════════════════════════════════════════════════════════════

PROBLÈME : "Port série introuvable"
SOLUTION : 
  - Windows : Vérifier Gestionnaire de périphériques
  - Linux : Vérifier /dev/ttyUSB* ou /dev/ttyACM*
  - Adapter DEVICENAME dans le code

PROBLÈME : "Servo ne répond pas"
SOLUTION :
  - Vérifier alimentation 12V branchée
  - Scanner les IDs avec Dynamixel Wizard
  - Vérifier le baudrate (1000000 par défaut)

PROBLÈME : "ModuleNotFoundError: No module named 'dynamixel_sdk'"
SOLUTION :
  - Réinstaller : pip install dynamixel-sdk
  - Vérifier version Python : python --version
  - Utiliser pip3 sous Linux/Mac

PROBLÈME : "Mouvements saccadés"
SOLUTION :
  - Augmenter le nombre de points (steps) dans generate_trajectory()
  - Utiliser generate_smooth_trajectory() au lieu de generate_linear_trajectory()
  - Réduire la vitesse des servos (paramètre speed)

PROBLÈME : "Cinématique inverse échoue"
SOLUTION :
  - Position hors workspace (trop loin ou trop proche)
  - Vérifier portée max = L1 + L2 + L3 = 11+11+11 = 33 cm
  - Tester avec position simple : (15, 10, 20)

PROBLÈME : "Interface graphique ne s'affiche pas"
SOLUTION :
  - Linux : sudo apt-get install python3-tk
  - Vérifier matplotlib : pip install matplotlib
  - Tester sans 3D en commentant la section create_3d_plot()


📖 EXERCICES PÉDAGOGIQUES
═══════════════════════════════════════════════════════════════════════════════

NIVEAU DÉBUTANT :
  ☐ Exercice 1 : Déplacer chaque servo individuellement
  ☐ Exercice 2 : Ramener le robot en position repos (0°)
  ☐ Exercice 3 : Enregistrer et rejouer 3 positions

NIVEAU INTERMÉDIAIRE :
  ☐ Exercice 4 : Faire décrire un carré dans le plan horizontal
  ☐ Exercice 5 : Atteindre 5 positions cibles données
  ☐ Exercice 6 : Générer une trajectoire linéaire de 50 points

NIVEAU AVANCÉ :
  ☐ Exercice 7 : Tracer un cercle de rayon 10 cm
  ☐ Exercice 8 : Pick & Place simplifié (A → B)
  ☐ Exercice 9 : Optimiser une trajectoire pour minimiser le temps
  ☐ Exercice 10 : Créer une interface personnalisée


🎓 EXPLOITATION PÉDAGOGIQUE PAR NIVEAU
═══════════════════════════════════════════════════════════════════════════════

COLLÈGE (4e-3e) :
  → Découverte de la programmation par blocs puis Python
  → Projets simples : dessiner son prénom, suivre un parcours
  → Notion de coordonnées cartésiennes et repères

LYCÉE (2nde à Terminale) :
  → Étude mathématique de la cinématique (trigonométrie)
  → Projets d'automatisation (tri, assemblage)
  → Asservissement et régulation

BTS (Systèmes Numériques, CRSA, ATI) :
  → Optimisation de trajectoires (temps, énergie)
  → Intégration capteurs (vision, force)
  → Contrôle avancé (jacobienne, planification)
  → Interface professionnelle (Qt, API REST)


📞 SUPPORT ET RESSOURCES
═══════════════════════════════════════════════════════════════════════════════

DOCUMENTATION OFFICIELLE :
  → Dynamixel SDK : https://emanual.robotis.com/
  → Python : https://docs.python.org/3/
  → NumPy : https://numpy.org/doc/
  → Matplotlib : https://matplotlib.org/

COMMUNAUTÉS :
  → Forum Robotis : https://community.robotis.com/
  → Stack Overflow : tag [dynamixel] [robotics]
  → Reddit : r/robotics

FORMATION GIPTIC :
  → Contact : [votre email]
  → Site web : [URL du GIPTIC]
  → Prochaine session : [date]


═══════════════════════════════════════════════════════════════════════════════
  Bonne formation !
  L'équipe GIPTIC - Robotique pédagogique
═══════════════════════════════════════════════════════════════════════════════

Version : 1.0
Date : Janvier 2026
Licence : Éducation nationale - Usage pédagogique
