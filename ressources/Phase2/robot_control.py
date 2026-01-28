#!/usr/bin/env python3
"""
===============================================================================
PROGRAMME DE CONTRÔLE - BRAS ROBOTIQUE 6 AXES DYNAMIXEL AX-12
===============================================================================

Auteur : Formation GIPTIC - Robotique Pédagogique
Date : Janvier 2026
Python : 3.8+

DESCRIPTION :
Ce programme permet le contrôle complet d'un bras robotique 6 axes équipé
de servomoteurs Dynamixel AX-12. Il fournit une interface graphique intuitive
pour le pilotage manuel, l'enregistrement de séquences, et la génération
de trajectoires.

BIBLIOTHÈQUES REQUISES :
- dynamixel_sdk : Communication avec les servomoteurs
- numpy : Calculs mathématiques (matrices, trigonométrie)
- matplotlib : Visualisation 3D du robot
- tkinter : Interface graphique

INSTALLATION :
pip install dynamixel-sdk numpy matplotlib

UTILISATION :
python robot_control.py

===============================================================================
"""

import tkinter as tk
from tkinter import ttk, messagebox, Frame
import math
import numpy as np
import threading
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass

# Importation conditionnelle pour Dynamixel SDK
try:
    from dynamixel_sdk import *
    DYNAMIXEL_SDK_AVAILABLE = True
except ImportError:
    DYNAMIXEL_SDK_AVAILABLE = False
    print("⚠️  Mode simulation : Dynamixel SDK non disponible")

# Importation conditionnelle pour matplotlib
try:
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from mpl_toolkits.mplot3d import Axes3D
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("⚠️  Visualisation 3D désactivée : matplotlib non disponible")


# ============================================================================
# STRUCTURES DE DONNÉES
# ============================================================================

@dataclass
class Point3D:
    """Représente un point dans l'espace 3D"""
    x: float
    y: float
    z: float
    
    def to_tuple(self) -> Tuple[float, float, float]:
        """Convertit en tuple (x, y, z)"""
        return (self.x, self.y, self.z)


# ============================================================================
# CLASSE PRINCIPALE : CONTRÔLEUR DU ROBOT
# ============================================================================

class RobotController:
    """
    Gère la communication avec les servomoteurs Dynamixel et les calculs
    de cinématique du robot 6 axes.
    
    CONFIGURATION MATÉRIELLE :
    - Port série : COM3 (Windows) ou /dev/ttyUSB0 (Linux)
    - Baudrate : 1 000 000 bps
    - Protocole : Dynamixel 1.0
    - IDs servos : 1 à 6
    
    PARAMÈTRES GÉOMÉTRIQUES (en cm) :
    - L1 : 11 cm (base → épaule)
    - L2 : 11 cm (épaule → coude)
    - L3 : 11 cm (coude → poignet)
    - L4 : 8 cm (poignet → effecteur)
    """
    
    # ========================================================================
    # CONSTANTES DE CONFIGURATION
    # ========================================================================
    
    # Configuration communication série
    DEVICENAME = "COM3"  # À adapter selon votre système
    BAUDRATE = 1000000   # 1 Mbps
    PROTOCOL_VERSION = 1.0
    
    # Adresses des registres Dynamixel AX-12
    ADDR_GOAL_POSITION = 0x1E      # Consigne de position (2 octets)
    ADDR_MOVING_SPEED = 0x20       # Vitesse de déplacement
    ADDR_PRESENT_POSITION = 0x24   # Position actuelle
    ADDR_PRESENT_LOAD = 0x28       # Charge actuelle
    ADDR_PRESENT_TEMPERATURE = 0x2B # Température
    ADDR_MOVING = 0x2E             # État de mouvement
    
    # Paramètres géométriques du robot (longueurs des segments en cm)
    L1 = 11.0  # Base → Épaule
    L2 = 11.0  # Épaule → Coude
    L3 = 11.0  # Coude → Poignet
    L4 = 8.0   # Poignet → Effecteur
    
    def __init__(self):
        """Initialise le contrôleur et configure la communication"""
        
        # Configuration des servomoteurs
        self.servo_ids = [1, 2, 3, 4, 5, 6]
        self.servo_names = {
            1: "Base (rotation)",
            2: "Épaule",
            3: "Coude",
            4: "Avant-bras",
            5: "Poignet",
            6: "Pince"
        }
        
        # État de connexion
        self.connected = False
        self.simulation_mode = True
        
        # Positions actuelles des servos (valeur Dynamixel 0-1023)
        # Position neutre = 512 (milieu de la plage)
        self.current_positions = [512] * 6
        
        # Historique des trajectoires enregistrées
        self.recorded_trajectory = []
        
        # Gestion du port série et des handlers Dynamixel
        self.port_handler = None
        self.packet_handler = None
        
        # Tentative de connexion au robot
        self._connect()
    
    # ========================================================================
    # MÉTHODES DE CONNEXION ET COMMUNICATION
    # ========================================================================
    
    def _connect(self):
        """
        Établit la connexion avec le contrôleur USB2AX/U2D2
        
        Si la connexion échoue, le programme bascule en mode simulation.
        """
        if not DYNAMIXEL_SDK_AVAILABLE:
            print("Mode simulation activé (SDK non disponible)")
            return
        
        try:
            # Initialisation du port série
            self.port_handler = PortHandler(self.DEVICENAME)
            self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)
            
            # Ouverture du port
            if self.port_handler.openPort():
                print(f"✅ Port {self.DEVICENAME} ouvert")
            else:
                raise Exception("Impossible d'ouvrir le port")
            
            # Configuration du baudrate
            if self.port_handler.setBaudRate(self.BAUDRATE):
                print(f"✅ Baudrate configuré : {self.BAUDRATE} bps")
            else:
                raise Exception("Impossible de configurer le baudrate")
            
            # Test de communication avec chaque servo
            print("\n🔍 Scan des servomoteurs...")
            for servo_id in self.servo_ids:
                position, comm_result, error = self.packet_handler.read2ByteTxRx(
                    self.port_handler, servo_id, self.ADDR_PRESENT_POSITION
                )
                
                if comm_result == COMM_SUCCESS:
                    print(f"   Servo {servo_id} ({self.servo_names[servo_id]}) : OK")
                    self.current_positions[servo_id - 1] = position
                else:
                    print(f"   ⚠️  Servo {servo_id} : Non détecté")
            
            self.connected = True
            self.simulation_mode = False
            print("\n✅ Robot connecté et opérationnel\n")
            
        except Exception as e:
            print(f"\n⚠️  Connexion impossible : {e}")
            print("Basculement en mode simulation\n")
            self.connected = False
            self.simulation_mode = True
    
    def move_servo(self, servo_id: int, angle: float, speed: int = 150):
        """
        Commande un servomoteur en position
        
        Args:
            servo_id : ID du servo (1 à 6)
            angle : Angle en degrés (-150° à +150°)
            speed : Vitesse de déplacement (0 à 1023)
        
        Conversion angle → position Dynamixel :
        - Position 0 = -150°
        - Position 512 = 0° (neutre)
        - Position 1023 = +150°
        
        Formule : position = 512 + (angle × 1024 / 300)
        """
        
        # Validation des paramètres
        if servo_id < 1 or servo_id > 6:
            print(f"❌ ID servo invalide : {servo_id}")
            return
        
        # Limitation de l'angle
        angle = max(-150, min(150, angle))
        
        # Conversion angle → position Dynamixel
        position = int(512 + angle * 1024 / 300)
        position = max(0, min(1023, position))
        
        if self.simulation_mode:
            # Mode simulation : mise à jour directe
            self.current_positions[servo_id - 1] = position
            print(f"🤖 [SIM] Servo {servo_id} → {angle:.1f}° (pos: {position})")
        else:
            # Mode réel : envoi de la commande
            try:
                # Écriture de la vitesse
                comm_result, error = self.packet_handler.write2ByteTxRx(
                    self.port_handler, servo_id, 
                    self.ADDR_MOVING_SPEED, speed
                )
                
                if comm_result != COMM_SUCCESS:
                    print(f"❌ Erreur vitesse servo {servo_id}")
                    return
                
                # Écriture de la position cible
                comm_result, error = self.packet_handler.write2ByteTxRx(
                    self.port_handler, servo_id, 
                    self.ADDR_GOAL_POSITION, position
                )
                
                if comm_result == COMM_SUCCESS:
                    self.current_positions[servo_id - 1] = position
                    print(f"✅ Servo {servo_id} → {angle:.1f}°")
                else:
                    print(f"❌ Erreur position servo {servo_id}")
                    
            except Exception as e:
                print(f"❌ Exception servo {servo_id} : {e}")
    
    def read_position(self, servo_id: int) -> int:
        """
        Lit la position actuelle d'un servomoteur
        
        Args:
            servo_id : ID du servo (1 à 6)
            
        Returns:
            Position Dynamixel (0-1023) ou -1 en cas d'erreur
        """
        if self.simulation_mode:
            return self.current_positions[servo_id - 1]
        
        try:
            position, comm_result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, self.ADDR_PRESENT_POSITION
            )
            
            if comm_result == COMM_SUCCESS:
                return position
            else:
                return -1
                
        except Exception as e:
            print(f"❌ Erreur lecture servo {servo_id} : {e}")
            return -1
    
    def get_servo_info(self, servo_id: int) -> dict:
        """
        Récupère les informations complètes d'un servomoteur
        
        Returns:
            dict avec clés : position, load, temperature, moving
        """
        info = {
            'position': 0,
            'load': 0,
            'temperature': 0,
            'moving': False
        }
        
        if self.simulation_mode:
            info['position'] = self.current_positions[servo_id - 1]
            return info
        
        try:
            # Position
            pos, _, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, self.ADDR_PRESENT_POSITION
            )
            info['position'] = pos
            
            # Charge
            load, _, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, self.ADDR_PRESENT_LOAD
            )
            info['load'] = load
            
            # Température
            temp, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, self.ADDR_PRESENT_TEMPERATURE
            )
            info['temperature'] = temp
            
            # État de mouvement
            moving, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, self.ADDR_MOVING
            )
            info['moving'] = (moving == 1)
            
        except Exception as e:
            print(f"❌ Erreur lecture info servo {servo_id} : {e}")
        
        return info
    
    # ========================================================================
    # CINÉMATIQUE DIRECTE
    # ========================================================================
    
    def position_to_angle(self, position: int) -> float:
        """
        Convertit une position Dynamixel en angle (degrés)
        
        Args:
            position : Valeur Dynamixel (0-1023)
            
        Returns:
            Angle en degrés (-150° à +150°)
        """
        return (position - 512) * 300 / 1024
    
    def forward_kinematics_full(self, angles: List[float]) -> List[np.ndarray]:
        """
        Calcule les positions de toutes les articulations du robot
        
        Args:
            angles : Liste de 6 angles en degrés [θ1, θ2, θ3, θ4, θ5, θ6]
            
        Returns:
            Liste de matrices de transformation 4×4 pour chaque articulation
            
        PRINCIPE :
        Utilise les transformations homogènes pour calculer la position
        de chaque segment du robot. Chaque matrice représente la pose
        (position + orientation) d'une articulation dans le repère de base.
        """
        
        # Conversion degrés → radians
        a = [math.radians(angle) for angle in angles]
        
        # Matrice identité 4×4 (repère de base)
        T_base = np.eye(4)
        
        # Liste des transformations cumulées
        transforms = [T_base.copy()]
        
        # ====================================================================
        # ARTICULATION 1 : BASE (rotation autour de Z)
        # ====================================================================
        T1 = np.array([
            [math.cos(a[0]), -math.sin(a[0]), 0, 0],
            [math.sin(a[0]),  math.cos(a[0]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        T_current = T_base @ T1
        transforms.append(T_current.copy())
        
        # ====================================================================
        # ARTICULATION 2 : ÉPAULE (rotation autour de Y + translation Z)
        # ====================================================================
        T2 = np.array([
            [math.cos(a[1]), 0, math.sin(a[1]), 0],
            [0, 1, 0, 0],
            [-math.sin(a[1]), 0, math.cos(a[1]), self.L1],
            [0, 0, 0, 1]
        ])
        T_current = T_current @ T2
        transforms.append(T_current.copy())
        
        # ====================================================================
        # ARTICULATION 3 : COUDE (rotation autour de Y + translation Z)
        # ====================================================================
        T3 = np.array([
            [math.cos(a[2]), 0, math.sin(a[2]), 0],
            [0, 1, 0, 0],
            [-math.sin(a[2]), 0, math.cos(a[2]), self.L2],
            [0, 0, 0, 1]
        ])
        T_current = T_current @ T3
        transforms.append(T_current.copy())
        
        # ====================================================================
        # ARTICULATION 4 : AVANT-BRAS (rotation autour de Y + translation Z)
        # ====================================================================
        T4 = np.array([
            [math.cos(a[3]), 0, math.sin(a[3]), 0],
            [0, 1, 0, 0],
            [-math.sin(a[3]), 0, math.cos(a[3]), self.L3],
            [0, 0, 0, 1]
        ])
        T_current = T_current @ T4
        transforms.append(T_current.copy())
        
        # ====================================================================
        # ARTICULATION 5 : POIGNET (rotation autour de Y + translation Z)
        # ====================================================================
        T5 = np.array([
            [math.cos(a[4]), 0, math.sin(a[4]), 0],
            [0, 1, 0, 0],
            [-math.sin(a[4]), 0, math.cos(a[4]), self.L4],
            [0, 0, 0, 1]
        ])
        T_current = T_current @ T5
        transforms.append(T_current.copy())
        
        # ====================================================================
        # ARTICULATION 6 : PINCE (rotation autour de Z)
        # ====================================================================
        T6 = np.array([
            [math.cos(a[5]), -math.sin(a[5]), 0, 0],
            [math.sin(a[5]),  math.cos(a[5]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        T_current = T_current @ T6
        transforms.append(T_current.copy())
        
        return transforms
    
    def get_end_effector_pose(self, angles: List[float]) -> Tuple[float, float, float, float]:
        """
        Calcule la position et l'orientation de l'effecteur terminal
        
        Args:
            angles : Liste de 6 angles en degrés
            
        Returns:
            (x, y, z, orientation_z) : Position en cm et orientation en degrés
        """
        transforms = self.forward_kinematics_full(angles)
        T_final = transforms[-1]
        
        # Extraction de la position
        x = T_final[0, 3]
        y = T_final[1, 3]
        z = T_final[2, 3]
        
        # Extraction de l'orientation (angle autour de Z)
        orientation_z = math.degrees(math.atan2(T_final[1, 0], T_final[0, 0]))
        
        return (x, y, z, orientation_z)
    
    # ========================================================================
    # CINÉMATIQUE INVERSE (MÉTHODE GÉOMÉTRIQUE SIMPLIFIÉE)
    # ========================================================================
    
    def inverse_kinematics(self, x: float, y: float, z: float) -> Optional[List[float]]:
        """
        Calcule les angles articulaires pour atteindre une position (x, y, z)
        
        Args:
            x, y, z : Coordonnées cibles en cm
            
        Returns:
            Liste de 6 angles en degrés ou None si position inatteignable
            
        MÉTHODE :
        Résolution géométrique pour un robot 3 axes planaire + rotations
        supplémentaires pour les 3 derniers axes.
        
        LIMITATIONS :
        - Ne gère pas l'évitement de singularités
        - Retourne une seule solution (plusieurs peuvent exister)
        - Workspace limité par les longueurs L1, L2, L3
        """
        
        # ====================================================================
        # ÉTAPE 1 : Calcul de l'angle de base (rotation autour de Z)
        # ====================================================================
        theta1 = math.degrees(math.atan2(y, x))
        
        # Distance horizontale projetée
        r = math.sqrt(x**2 + y**2)
        
        # ====================================================================
        # ÉTAPE 2 : Résolution du problème planaire 2D (bras articulé)
        # ====================================================================
        
        # Hauteur ajustée (par rapport à la base)
        h = z - self.L1
        
        # Distance du point cible dans le plan (r, h)
        d = math.sqrt(r**2 + h**2)
        
        # Vérification : point atteignable ?
        max_reach = self.L2 + self.L3
        min_reach = abs(self.L2 - self.L3)
        
        if d > max_reach or d < min_reach:
            print(f"❌ Position inatteignable : distance = {d:.1f} cm")
            print(f"   Portée min/max : {min_reach:.1f} - {max_reach:.1f} cm")
            return None
        
        # Loi des cosinus pour trouver theta3 (angle au coude)
        cos_theta3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta3 = max(-1, min(1, cos_theta3))  # Sécurité numérique
        
        theta3 = math.degrees(math.acos(cos_theta3))
        
        # Calcul de theta2 (angle à l'épaule)
        alpha = math.atan2(h, r)
        beta = math.acos((self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d))
        
        theta2 = math.degrees(alpha + beta)
        
        # ====================================================================
        # ÉTAPE 3 : Angles des axes terminaux (simplifiés)
        # ====================================================================
        
        # Orientation de l'effecteur (parallèle au sol)
        theta4 = -(theta2 + theta3)
        
        # Poignet et pince en position neutre
        theta5 = 0
        theta6 = 0
        
        angles = [theta1, theta2, theta3, theta4, theta5, theta6]
        
        # Vérification finale
        print(f"✅ Solution trouvée : θ = {[f'{a:.1f}°' for a in angles]}")
        
        return angles
    
    # ========================================================================
    # GÉNÉRATION DE TRAJECTOIRES
    # ========================================================================
    
    def generate_linear_trajectory(self, start: Point3D, end: Point3D, steps: int = 50) -> List[Point3D]:
        """
        Génère une trajectoire linéaire entre deux points
        
        Args:
            start : Point de départ
            end : Point d'arrivée
            steps : Nombre de points intermédiaires
            
        Returns:
            Liste de points 3D
        """
        trajectory = []
        
        for i in range(steps):
            t = i / (steps - 1)  # Paramètre de 0 à 1
            
            x = start.x + t * (end.x - start.x)
            y = start.y + t * (end.y - start.y)
            z = start.z + t * (end.z - start.z)
            
            trajectory.append(Point3D(x, y, z))
        
        return trajectory
    
    def generate_smooth_trajectory(self, start: Point3D, end: Point3D, steps: int = 50) -> List[Point3D]:
        """
        Génère une trajectoire avec profil de vitesse adouci (sinusoïdal)
        
        Avantage : Accélération/décélération progressive → mouvement fluide
        """
        trajectory = []
        
        for i in range(steps):
            t_raw = i / (steps - 1)
            
            # Profil sinusoïdal : accélération douce
            t = (1 - math.cos(t_raw * math.pi)) / 2
            
            x = start.x + t * (end.x - start.x)
            y = start.y + t * (end.y - start.y)
            z = start.z + t * (end.z - start.z)
            
            trajectory.append(Point3D(x, y, z))
        
        return trajectory
    
    def execute_trajectory(self, trajectory: List[Point3D], delay: float = 0.05):
        """
        Exécute une trajectoire point par point
        
        Args:
            trajectory : Liste de points 3D
            delay : Délai entre chaque point (secondes)
        """
        print(f"\n🚀 Exécution trajectoire ({len(trajectory)} points)...")
        
        for i, point in enumerate(trajectory):
            # Calcul de la cinématique inverse
            angles = self.inverse_kinematics(point.x, point.y, point.z)
            
            if angles is None:
                print(f"❌ Point {i+1} inatteignable : {point}")
                continue
            
            # Commande des servos
            for servo_id in range(1, 6):  # Axes 1-5 (pas la pince)
                self.move_servo(servo_id, angles[servo_id - 1], speed=200)
            
            time.sleep(delay)
        
        print("✅ Trajectoire terminée\n")
    
    # ========================================================================
    # TEACH-IN (ENREGISTREMENT/LECTURE DE SÉQUENCES)
    # ========================================================================
    
    def record_current_position(self):
        """Enregistre la position actuelle dans la trajectoire"""
        
        # Lecture des positions
        current_angles = []
        for servo_id in range(1, 7):
            pos = self.read_position(servo_id)
            angle = self.position_to_angle(pos)
            current_angles.append(angle)
        
        # Ajout à l'historique
        self.recorded_trajectory.append(current_angles.copy())
        
        print(f"📝 Position {len(self.recorded_trajectory)} enregistrée")
    
    def replay_trajectory(self, speed: int = 150):
        """Rejoue la trajectoire enregistrée"""
        
        if not self.recorded_trajectory:
            print("❌ Aucune trajectoire enregistrée")
            return
        
        print(f"\n▶️  Lecture de {len(self.recorded_trajectory)} positions...")
        
        for i, angles in enumerate(self.recorded_trajectory):
            print(f"   Position {i+1}/{len(self.recorded_trajectory)}")
            
            for servo_id in range(1, 7):
                self.move_servo(servo_id, angles[servo_id - 1], speed)
            
            time.sleep(1.0)  # Pause entre positions
        
        print("✅ Lecture terminée\n")
    
    def clear_trajectory(self):
        """Efface la trajectoire enregistrée"""
        self.recorded_trajectory = []
        print("🗑️  Trajectoire effacée")
    
    # ========================================================================
    # UTILITAIRES
    # ========================================================================
    
    def home_position(self):
        """Ramène le robot en position de repos (tous servos à 0°)"""
        print("\n🏠 Retour position repos...")
        
        for servo_id in range(1, 7):
            self.move_servo(servo_id, 0, speed=100)
        
        time.sleep(2)
        print("✅ Position repos atteinte\n")
    
    def disconnect(self):
        """Ferme la connexion série"""
        if self.connected and self.port_handler:
            self.port_handler.closePort()
            print("🔌 Robot déconnecté")


# ============================================================================
# INTERFACE GRAPHIQUE
# ============================================================================

class RobotGUI:
    """
    Interface graphique Tkinter pour le contrôle du robot
    
    FONCTIONNALITÉS :
    - Contrôle manuel avec sliders (6 axes)
    - Visualisation 3D en temps réel
    - Affichage des coordonnées cartésiennes
    - Boutons teach-in (enregistrer/rejouer)
    - Onglet cinématique inverse
    - Monitoring des servos (position, charge, température)
    """
    
    def __init__(self, robot: RobotController):
        """Initialise l'interface graphique"""
        
        self.robot = robot
        self.root = tk.Tk()
        self.root.title("Contrôle Robot 6 Axes - GIPTIC Formation")
        self.root.geometry("1400x800")
        
        # Conteneurs principaux
        self.sliders = {}
        self.create_layout()
        
        # Timer pour mise à jour périodique
        self.update_timer()
    
    def create_layout(self):
        """Crée l'organisation générale de l'interface"""
        
        # ====================================================================
        # FRAME GAUCHE : CONTRÔLES
        # ====================================================================
        
        frame_left = tk.Frame(self.root, width=400, bg="#F0F0F0")
        frame_left.pack(side=tk.LEFT, fill=tk.BOTH, padx=10, pady=10)
        
        # Titre
        title = tk.Label(
            frame_left, 
            text="🤖 CONTRÔLE MANUEL",
            font=("Arial", 16, "bold"),
            bg="#2563EB",
            fg="white",
            pady=10
        )
        title.pack(fill=tk.X)
        
        # Zone des sliders
        slider_frame = tk.Frame(frame_left, bg="#F0F0F0")
        slider_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        self.create_sliders(slider_frame)
        
        # Boutons teach-in
        btn_frame = tk.Frame(frame_left, bg="#F0F0F0")
        btn_frame.pack(fill=tk.X, pady=10)
        
        tk.Button(
            btn_frame, 
            text="📝 Enregistrer position",
            command=self.record_position,
            bg="#10B981",
            fg="white",
            font=("Arial", 10, "bold"),
            height=2
        ).pack(fill=tk.X, pady=2)
        
        tk.Button(
            btn_frame, 
            text="▶️  Rejouer séquence",
            command=self.replay_sequence,
            bg="#3B82F6",
            fg="white",
            font=("Arial", 10, "bold"),
            height=2
        ).pack(fill=tk.X, pady=2)
        
        tk.Button(
            btn_frame, 
            text="🗑️  Effacer séquence",
            command=self.clear_sequence,
            bg="#EF4444",
            fg="white",
            font=("Arial", 10, "bold"),
            height=2
        ).pack(fill=tk.X, pady=2)
        
        tk.Button(
            btn_frame, 
            text="🏠 Position repos",
            command=self.home,
            bg="#6B7280",
            fg="white",
            font=("Arial", 10, "bold"),
            height=2
        ).pack(fill=tk.X, pady=2)
        
        # Affichage coordonnées
        coord_frame = tk.Frame(frame_left, bg="#DBEAFE", relief=tk.RIDGE, bd=2)
        coord_frame.pack(fill=tk.X, pady=10)
        
        tk.Label(
            coord_frame,
            text="📍 Position effecteur",
            font=("Arial", 12, "bold"),
            bg="#DBEAFE"
        ).pack()
        
        self.label_xyz = tk.Label(
            coord_frame,
            text="X: 0.0  Y: 0.0  Z: 0.0",
            font=("Courier", 14, "bold"),
            bg="#DBEAFE",
            fg="#1E40AF"
        )
        self.label_xyz.pack(pady=5)
        
        # ====================================================================
        # FRAME DROITE : VISUALISATION 3D
        # ====================================================================
        
        frame_right = tk.Frame(self.root)
        frame_right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        if MATPLOTLIB_AVAILABLE:
            self.create_3d_plot(frame_right)
        else:
            tk.Label(
                frame_right,
                text="⚠️  Visualisation 3D non disponible\n(matplotlib requis)",
                font=("Arial", 14),
                fg="red"
            ).pack(expand=True)
    
    def create_sliders(self, parent):
        """Crée les sliders de contrôle pour chaque servomoteur"""
        
        servo_names_short = ["Base", "Épaule", "Coude", "Av-bras", "Poignet", "Pince"]
        
        for sid in range(1, 7):
            frame = tk.Frame(parent, bg="#F0F0F0")
            frame.pack(fill=tk.X, pady=5)
            
            # Label
            label = tk.Label(
                frame, 
                text=f"M{sid} {servo_names_short[sid-1]}", 
                width=12, 
                anchor='w', 
                font=('Arial', 10, 'bold'),
                bg="#F0F0F0"
            )
            label.pack(side=tk.LEFT, padx=5)
            
            # Slider
            slider = tk.Scale(
                frame, 
                from_=-150, 
                to=150, 
                orient=tk.HORIZONTAL, 
                length=250,
                resolution=1, 
                command=lambda val, s=sid: self.on_slider_change(val, s),
                showvalue=0,
                bg="#FFFFFF",
                troughcolor="#3B82F6",
                highlightthickness=0
            )
            slider.set(0)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            
            # Valeur numérique
            value_label = tk.Label(
                frame, 
                text="0°", 
                width=6, 
                font=('Courier', 10, 'bold'),
                bg="#F0F0F0",
                fg="#1E40AF"
            )
            value_label.pack(side=tk.LEFT, padx=5)
            
            # Indicateur de position
            indicator = tk.Canvas(frame, width=20, height=20, bg="#F0F0F0", highlightthickness=0)
            indicator.pack(side=tk.LEFT, padx=5)
            circle = indicator.create_oval(2, 2, 18, 18, fill="#10B981", outline="")
            
            self.sliders[sid] = {
                'slider': slider, 
                'label': value_label,
                'indicator': indicator,
                'circle': circle
            }
    
    def create_3d_plot(self, parent):
        """Crée la visualisation 3D du robot"""
        
        # Figure matplotlib
        self.fig = plt.Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Configuration des axes
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        self.ax.set_zlim(0, 50)
        self.ax.set_xlabel("X (cm)", fontsize=10)
        self.ax.set_ylabel("Y (cm)", fontsize=10)
        self.ax.set_zlabel("Z (cm)", fontsize=10)
        self.ax.set_title("Visualisation 3D", fontsize=12, weight='bold')
        self.ax.view_init(elev=20, azim=45)
        
        # Intégration dans Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Dessin initial
        self.update_3d_plot()
    
    def update_3d_plot(self):
        """Met à jour la visualisation 3D"""
        
        if not MATPLOTLIB_AVAILABLE:
            return
        
        self.ax.clear()
        
        # Configuration
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        self.ax.set_zlim(0, 50)
        self.ax.set_xlabel("X (cm)")
        self.ax.set_ylabel("Y (cm)")
        self.ax.set_zlabel("Z (cm)")
        self.ax.view_init(elev=20, azim=45)
        
        # Récupération des angles
        angles = [self.sliders[i]['slider'].get() for i in range(1, 7)]
        
        # Calcul des positions
        transforms = self.robot.forward_kinematics_full(angles)
        points = [T[0:3, 3] for T in transforms]
        
        # Tracé des segments
        colors = ["#2563EB", "#3B82F6", "#60A5FA", "#93C5FD", "#BFDBFE", "#DBEAFE"]
        
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i + 1]
            self.ax.plot(
                [p1[0], p2[0]], 
                [p1[1], p2[1]], 
                [p1[2], p2[2]], 
                color=colors[i % len(colors)], 
                linewidth=4,
                marker='o',
                markersize=6
            )
        
        # Tracé de la base
        self.ax.scatter([0], [0], [0], color='red', s=100, marker='s', label="Base")
        
        # Rafraîchissement
        self.canvas.draw()
    
    def on_slider_change(self, value, servo_id):
        """Callback appelé quand un slider est déplacé"""
        
        angle = float(value)
        
        # Mise à jour du label
        self.sliders[servo_id]['label'].config(text=f"{angle:.0f}°")
        
        # Commande du servo
        self.robot.move_servo(servo_id, angle, speed=200)
        
        # Mise à jour de l'affichage
        self.update_display()
    
    def update_display(self):
        """Met à jour l'affichage (coordonnées + 3D)"""
        
        # Calcul position effecteur
        angles = [self.sliders[i]['slider'].get() for i in range(1, 6)]
        angles.append(0)  # Pince
        
        x, y, z, _ = self.robot.get_end_effector_pose(angles)
        
        # Affichage
        self.label_xyz.config(text=f"X: {x:.1f}  Y: {y:.1f}  Z: {z:.1f}")
        
        # Mise à jour 3D
        self.update_3d_plot()
    
    def update_timer(self):
        """Timer pour mise à jour périodique"""
        
        # Rafraîchissement toutes les 100 ms
        self.root.after(100, self.update_timer)
    
    # ========================================================================
    # CALLBACKS BOUTONS
    # ========================================================================
    
    def record_position(self):
        """Enregistre la position actuelle"""
        self.robot.record_current_position()
        messagebox.showinfo("✅ Enregistré", 
                          f"Position {len(self.robot.recorded_trajectory)} enregistrée")
    
    def replay_sequence(self):
        """Rejoue la séquence enregistrée"""
        if not self.robot.recorded_trajectory:
            messagebox.showwarning("⚠️ Attention", "Aucune séquence enregistrée")
            return
        
        # Exécution dans un thread séparé
        threading.Thread(target=self.robot.replay_trajectory, daemon=True).start()
    
    def clear_sequence(self):
        """Efface la séquence"""
        self.robot.clear_trajectory()
        messagebox.showinfo("🗑️ Effacé", "Séquence effacée")
    
    def home(self):
        """Ramène en position repos"""
        threading.Thread(target=self.robot.home_position, daemon=True).start()
    
    def run(self):
        """Lance la boucle principale de l'interface"""
        self.root.mainloop()


# ============================================================================
# POINT D'ENTRÉE PRINCIPAL
# ============================================================================

def main():
    """Fonction principale du programme"""
    
    print("=" * 70)
    print("  CONTRÔLE BRAS ROBOTIQUE 6 AXES - FORMATION GIPTIC")
    print("=" * 70)
    print()
    
    # Initialisation du robot
    robot = RobotController()
    
    # Lancement de l'interface graphique
    gui = RobotGUI(robot)
    gui.run()
    
    # Déconnexion propre
    robot.disconnect()


if __name__ == "__main__":
    main()
