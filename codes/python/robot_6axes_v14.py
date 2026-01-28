import sys
import os

# ============================================================================
# FIX POUR PYINSTALLER EN MODE WINDOWED
# ============================================================================
# En mode --windowed, sys.stdout et sys.stderr sont None
# Solution : Créer des objets de remplacement avant tout import

class DummyStream:
    """Objet factice pour remplacer stdout/stderr en mode windowed"""
    def write(self, text):
        pass  # Ne rien faire, ignorer l'écriture
    
    def flush(self):
        pass  # Ne rien faire
    
    def isatty(self):
        return False

# Appliquer le fix AVANT tous les imports de bibliothèques
if sys.stdout is None:
    sys.stdout = DummyStream()
if sys.stderr is None:
    sys.stderr = DummyStream()

# Maintenant on peut importer les autres modules en toute sécurité
import tkinter as tk
import numpy as np
import math
import time
import csv
from tkinter import filedialog, simpledialog, messagebox
from dynamixel_sdk import *
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import serial.tools.list_ports  # Pour lister les ports COM

# ============================================================================
# CLASSE PRINCIPALE DU ROBOT
# ============================================================================

class Robot6Axes:
    """Classe principale pour gérer le robot 6 axes avec Dynamixel AX-12"""
    
    def __init__(self, port_name="COM6"):
        # Paramètres géométriques (cm)
        self.L1 = 13.0  # Base
        self.L2 = 11.0  # Bras 1
        self.L3 = 11.0  # Bras 2
        self.L4 = 17.0  # Poignet + pince (valeur qui fonctionnait)
        
        # Configuration Dynamixel
        self.DEVICENAME = port_name
        self.BAUDRATE = 1000000
        self.PROTOCOL_VERSION = 1.0
        
        # Adresses mémoire
        self.ADDR_GOAL_POSITION = 0x1E
        self.ADDR_PRESENT_POSITION = 0x24
        self.ADDR_MOVING = 0x2E
        self.ADDR_MOVING_SPEED = 0x20
        self.ADDR_TORQUE_ENABLE = 0x18
        
        # Servos
        self.servo_ids = [1, 2, 3, 4, 5, 6]
        self.servo_names = {
            1: "Base",
            2: "Épaule",
            3: "Coude",
            4: "Poignet",
            5: "Rotation pince",
            6: "Pince"
        }
        
        # Initialisation communication
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        if not self.portHandler.openPort():
            raise Exception("❌ Port série inaccessible")
        
        self.portHandler.setBaudRate(self.BAUDRATE)
        print("✅ Connexion établie avec les servos")
        
        # Angles courants
        self.current_angles = [0.0] * 6
        
    def __del__(self):
        """Fermeture propre du port série"""
        if hasattr(self, 'portHandler'):
            self.portHandler.closePort()
    
    # ------------------------------------------------------------------------
    # COMMUNICATION DYNAMIXEL
    # ------------------------------------------------------------------------
    
    def angle_to_position(self, angle_deg):
        """Conversion angle → position Dynamixel"""
        return int((angle_deg + 150) * 1023 / 300)
    
    def position_to_angle(self, pos):
        """Conversion position Dynamixel → angle"""
        return (pos * 300 / 1023) - 150
    
    def read_servo_position(self, servo_id):
        """Lecture position d'un servo"""
        pos, _, _ = self.packetHandler.read2ByteTxRx(
            self.portHandler, servo_id, self.ADDR_PRESENT_POSITION
        )
        return self.position_to_angle(pos)
    
    def is_servo_moving(self, servo_id):
        """Vérifie si un servo est en mouvement"""
        moving, _, _ = self.packetHandler.read1ByteTxRx(
            self.portHandler, servo_id, self.ADDR_MOVING
        )
        return moving == 1
    
    def wait_for_all_servos(self):
        """Attend que tous les servos aient fini leur mouvement"""
        time.sleep(0.05)
        while any(self.is_servo_moving(sid) for sid in self.servo_ids):
            time.sleep(0.05)
    
    def move_servo(self, servo_id, angle_deg, speed=100):
        """Déplace un servo à un angle donné avec une vitesse"""
        pos = self.angle_to_position(angle_deg)
        self.packetHandler.write2ByteTxRx(
            self.portHandler, servo_id, self.ADDR_GOAL_POSITION, pos
        )
        self.packetHandler.write2ByteTxRx(
            self.portHandler, servo_id, self.ADDR_MOVING_SPEED, speed
        )
        self.current_angles[servo_id - 1] = angle_deg
    
    def move_all_servos(self, angles, speed=100):
        """Déplace tous les servos simultanément"""
        for i, angle in enumerate(angles):
            self.move_servo(i + 1, angle, speed)
    
    # ------------------------------------------------------------------------
    # CINÉMATIQUE DIRECTE (Matrices de transformation homogènes)
    # ------------------------------------------------------------------------
    
    def dh_matrix(self, theta, d, a, alpha):
        """
        Calcule la matrice de transformation de Denavit-Hartenberg
        
        θ (theta): angle de rotation autour de z
        d: translation le long de z
        a: translation le long de x
        α (alpha): rotation autour de x
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct,    -st*ca,  st*sa,   a*ct],
            [st,     ct*ca, -ct*sa,   a*st],
            [0,      sa,     ca,      d],
            [0,      0,      0,       1]
        ])
    
    def forward_kinematics_full(self, angles):
        """
        Calcule la cinématique directe complète avec toutes les positions
        
        Retourne : liste des matrices de transformation pour chaque articulation
        """
        t1, t2, t3, t4, t5, t6 = [np.radians(a) for a in angles]
        
        # BASE (origine du robot)
        T0 = np.eye(4)
        
        # MOTEUR 1 : Base (rotation autour de Z global)
        T0_1 = self.dh_matrix(t1, self.L1, 0, np.pi/2)
        T1 = T0_1
        
        # MOTEUR 2 : Épaule (rotation dans plan vertical)
        T1_2 = self.dh_matrix(t2 + np.pi/2, 0, self.L2, 0)
        T2 = T1 @ T1_2
        
        # MOTEUR 3 : Coude (rotation dans plan vertical)
        T2_3 = self.dh_matrix(t3, 0, self.L3, 0)
        T3 = T2 @ T2_3
        
        # MOTEUR 4 : Poignet (rotation dans plan vertical)
        T3_4 = self.dh_matrix(t4, 0, self.L4, 0)
        T4 = T3 @ T3_4
        
        # MOTEUR 5 : Rotation pince autour de l'axe X LOCAL (axe rouge)
        # Offset de +90° pour que la pince pointe vers le haut en position 0
        t5_with_offset = t5 + np.pi/2
        
        # Matrice de rotation autour de X (axe rouge reste fixe)
        Rx = np.array([
            [1, 0, 0, 0],
            [0, np.cos(t5_with_offset), -np.sin(t5_with_offset), 0],
            [0, np.sin(t5_with_offset), np.cos(t5_with_offset), 0],
            [0, 0, 0, 1]
        ])
        T4_5 = Rx
        T5 = T4 @ T4_5
        
        # MOTEUR 6 : Ouverture/fermeture
        T5_6 = np.eye(4)
        T6 = T5 @ T5_6
        
        return [T0, T1, T2, T3, T4, T5, T6]
    
    def get_end_effector_pose(self, angles):
        """Retourne la position et orientation de l'effecteur final (avec correction d'affichage)"""
        transforms = self.forward_kinematics_full(angles)
        T_end = transforms[-1]
        
        # Position brute
        x_raw, y_raw, z_raw = T_end[0:3, 3]
        
        # Application de la transformation forward pour l'affichage
        # (inverse de apply_inverse_tilt utilisé dans la cinématique inverse)
        x, y, z = self.apply_forward_tilt(x_raw, y_raw, z_raw)
        
        # Orientation (vecteur direction)
        direction = T_end[0:3, 2]  # axe Z du repère final
        
        return x, y, z, direction
    
    # ------------------------------------------------------------------------
    # CINÉMATIQUE INVERSE (Méthode géométrique simplifiée)
    # ------------------------------------------------------------------------
    
    def apply_inverse_tilt(self, x, y, z):
        """Ancienne correction d'inclinaison supprimée (causait bug Z pour X négatifs)"""
        # Retourner directement sans transformation
        return x, y, z
    
    def apply_forward_tilt(self, x, y, z):
        """Ancienne correction d'inclinaison supprimée (causait bug Z pour X négatifs)"""
        # Retourner directement sans transformation
        return x, y, z
    
    def inverse_kinematics(self, x, y, z):
        """
        Calcule la cinématique inverse pour atteindre (x, y, z)
        
        Retourne: [θ1, θ2, θ3] ou None si hors d'atteinte
        """
        try:
            # Position du poignet T3
            # La pince s'étend vers le bas, donc le poignet est PLUS HAUT que z
            z_wrist = z + self.L4
            
            # Angle de base (rotation autour de Z)
            t1 = np.arctan2(y, x)
            
            # Distance horizontale depuis l'axe Z
            r = np.sqrt(x**2 + y**2)
            
            # Hauteur relative du poignet par rapport à la base
            dz = z_wrist - self.L1
            
            # Distance 2D dans le plan vertical
            d = np.sqrt(r**2 + dz**2)
            
            # Vérification atteignabilité (avec tolérance pour erreurs d'arrondi)
            if d > (self.L2 + self.L3) + 0.001:
                return None
            
            if d < abs(self.L2 - self.L3) - 0.001:
                return None
            
            # Loi des cosinus pour θ3
            D = (r**2 + dz**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
            
            # Clamping pour éviter les erreurs d'arrondi
            D = np.clip(D, -1.0, 1.0)
            
            # Configuration coude en bas (elbow-down)
            t3 = np.arctan2(-np.sqrt(1 - D**2), D)
            
            # Cas spécial : position verticale (r ≈ 0)
            if r < 0.001:
                # Position verticale : t2 dépend uniquement de la hauteur
                if dz > 0:
                    # Au-dessus de la base : bras tendus vers le haut
                    t2 = np.arctan2(dz, 0.001) - np.arctan2(self.L3 * np.sin(t3), 
                                                             self.L2 + self.L3 * np.cos(t3))
                else:
                    # En dessous (ne devrait pas arriver)
                    t2 = 0
            else:
                # Cas général : calcul standard
                t2 = np.arctan2(dz, r) - np.arctan2(self.L3 * np.sin(t3), 
                                                     self.L2 + self.L3 * np.cos(t3))
            
            # Conversion en degrés
            result = [
                np.degrees(t1),
                np.degrees(t2) - 90,  # Offset pour correspondre au modèle
                np.degrees(t3)
            ]
            
            return result
            
        except Exception as e:
            print(f"   [IK] ❌ Erreur: {e}")
            return None
    
    # ------------------------------------------------------------------------
    # INTERPOLATION DE TRAJECTOIRE
    # ------------------------------------------------------------------------
    
    def interpolate_trajectory(self, positions, num_points=50, method='cubic'):
        """
        Interpole une trajectoire entre plusieurs positions
        
        Args:
            positions: liste de positions [angle1, angle2, ..., angle6]
            num_points: nombre de points intermédiaires
            method: 'linear', 'cubic', ou 'quintic'
        
        Returns:
            Tableau numpy de positions interpolées
        """
        if len(positions) < 2:
            return np.array(positions)
        
        positions = np.array(positions)
        n_waypoints = len(positions)
        
        # Ajuster la méthode selon le nombre de waypoints
        if n_waypoints == 2:
            # Avec 2 points, seulement linéaire possible
            actual_method = 'linear'
        elif n_waypoints == 3:
            # Avec 3 points, quadratique ou linéaire
            actual_method = 'quadratic' if method in ['cubic', 'quintic'] else 'linear'
        elif n_waypoints >= 4 and method == 'cubic':
            actual_method = 'cubic'
        elif n_waypoints >= 6 and method == 'quintic':
            actual_method = 'quintic'
        else:
            # Par défaut
            actual_method = 'linear'
        
        # Temps normalisé pour chaque waypoint
        t_waypoints = np.linspace(0, 1, n_waypoints)
        
        # Temps pour les points interpolés
        t_interp = np.linspace(0, 1, num_points)
        
        # Interpolation pour chaque axe
        interpolated = np.zeros((num_points, 6))
        
        for axis in range(6):
            f = interp1d(t_waypoints, positions[:, axis], kind=actual_method)
            interpolated[:, axis] = f(t_interp)
        
        return interpolated
    
    def calculate_velocity_profile(self, trajectory, max_speed=200, 
                                   acceleration_ratio=0.3):
        """
        Calcule un profil de vitesse trapézoïdal pour une trajectoire
        
        Args:
            trajectory: positions interpolées
            max_speed: vitesse maximale (1-1023)
            acceleration_ratio: ratio du temps d'accélération/décélération
        
        Returns:
            Liste de vitesses pour chaque point
        """
        n_points = len(trajectory)
        speeds = np.zeros(n_points)
        
        # Zones d'accélération/décélération
        accel_points = int(n_points * acceleration_ratio)
        
        for i in range(n_points):
            if i < accel_points:
                # Phase d'accélération
                speeds[i] = max_speed * (i / accel_points)
            elif i > n_points - accel_points:
                # Phase de décélération
                speeds[i] = max_speed * ((n_points - i) / accel_points)
            else:
                # Vitesse constante
                speeds[i] = max_speed
        
        # Vitesse minimale pour éviter des mouvements trop lents
        speeds = np.maximum(speeds, 50)
        
        return speeds.astype(int)


# ============================================================================
# INTERFACE GRAPHIQUE
# ============================================================================

class RobotGUI:
    """Interface graphique pour contrôler le robot"""
    
    def __init__(self, robot):
        self.robot = robot
        self.recorded_positions = []
        self.current_speed = 200  # Vitesse par défaut
        
        # Variables pour le clignotement des voyants
        self.blink_state = False  # État du clignotement (True = allumé, False = éteint)
        self.indicators_status = {'x': True, 'y': True, 'z': True}  # État de chaque voyant
        self.blink_timer = None  # Timer pour le clignotement
        
        # Fenêtre principale (taille réduite)
        self.root = tk.Tk()
        self.root.title("Bras Robot 6 Axes - Version Améliorée")
        self.root.geometry("1100x750")  # Taille fixe pour interface compacte
        
        # Frame principale avec 2 colonnes
        main_container = tk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Colonne gauche : contrôles (largeur réduite)
        left_panel = tk.Frame(main_container, width=350, bg='#f0f0f0')
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
        left_panel.pack_propagate(False)
        
        # Colonne droite : visualisation 3D
        right_panel = tk.Frame(main_container)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # === CURSEUR DE VITESSE ===
        speed_frame = tk.LabelFrame(left_panel, text="⚡ Vitesse", 
                                    font=('Arial', 10, 'bold'), padx=5, pady=5)
        speed_frame.pack(fill=tk.X, padx=5, pady=5)
        
        speed_control = tk.Frame(speed_frame)
        speed_control.pack(fill=tk.X)
        
        self.speed_slider = tk.Scale(
            speed_control,
            from_=50, to=500,
            orient=tk.HORIZONTAL,
            length=200,
            command=self.on_speed_change,
            showvalue=0
        )
        self.speed_slider.set(200)
        self.speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.speed_label = tk.Label(speed_control, text="200", 
                                    font=('Arial', 10, 'bold'), 
                                    fg='#2196F3', width=4)
        self.speed_label.pack(side=tk.LEFT, padx=3)
        
        # Boutons rapides vitesse
        speed_btns = tk.Frame(speed_frame)
        speed_btns.pack(fill=tk.X, pady=2)
        for label, speed in [("🐌", 100), ("🚶", 200), ("🏃", 350), ("⚡", 500)]:
            tk.Button(speed_btns, text=label, width=4,
                     command=lambda s=speed: self.set_speed(s)).pack(
                         side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        
        # === SLIDERS SERVOS ===
        sliders_frame = tk.LabelFrame(left_panel, text="🎛️ Contrôles", 
                                      font=('Arial', 10, 'bold'), padx=5, pady=5)
        sliders_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.sliders = {}
        self.create_sliders(sliders_frame)
        
        # Bouton Reset
        tk.Button(sliders_frame, text="🔄 Reset", command=self.reset,
                 bg='#FF9800', fg='white', font=('Arial', 9, 'bold')).pack(
                     fill=tk.X, pady=3)
        
        # === AFFICHAGE POSITION + 3 VOYANTS (X, Y, Z) ===
        position_frame = tk.Frame(left_panel, bg='#f0f0f0')
        position_frame.pack(pady=5)
        
        # Label des coordonnées
        self.label_xyz = tk.Label(position_frame, 
                                  text="X: 0  Y: 0  Z: 0", 
                                  font=("Arial", 10, 'bold'), fg="blue", bg='#f0f0f0')
        self.label_xyz.pack()
        
        # Frame pour les 3 voyants
        indicators_frame = tk.Frame(position_frame, bg='#f0f0f0')
        indicators_frame.pack(pady=2)
        
        # Voyant X
        x_frame = tk.Frame(indicators_frame, bg='#f0f0f0')
        x_frame.pack(side=tk.LEFT, padx=8)
        tk.Label(x_frame, text="X", font=("Arial", 7), bg='#f0f0f0').pack()
        self.indicator_x = tk.Canvas(x_frame, width=15, height=15, 
                                    bg='#f0f0f0', highlightthickness=0)
        self.indicator_x.pack()
        self.circle_x = self.indicator_x.create_oval(2, 2, 13, 13, 
                                                     fill='green', outline='darkgreen', width=1)
        
        # Voyant Y
        y_frame = tk.Frame(indicators_frame, bg='#f0f0f0')
        y_frame.pack(side=tk.LEFT, padx=8)
        tk.Label(y_frame, text="Y", font=("Arial", 7), bg='#f0f0f0').pack()
        self.indicator_y = tk.Canvas(y_frame, width=15, height=15, 
                                    bg='#f0f0f0', highlightthickness=0)
        self.indicator_y.pack()
        self.circle_y = self.indicator_y.create_oval(2, 2, 13, 13, 
                                                     fill='green', outline='darkgreen', width=1)
        
        # Voyant Z
        z_frame = tk.Frame(indicators_frame, bg='#f0f0f0')
        z_frame.pack(side=tk.LEFT, padx=8)
        tk.Label(z_frame, text="Z", font=("Arial", 7), bg='#f0f0f0').pack()
        self.indicator_z = tk.Canvas(z_frame, width=15, height=15, 
                                    bg='#f0f0f0', highlightthickness=0)
        self.indicator_z.pack()
        self.circle_z = self.indicator_z.create_oval(2, 2, 13, 13, 
                                                     fill='green', outline='darkgreen', width=1)
        
        # === VISUALISATION 3D ===
        self.fig = plt.figure(figsize=(7, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # === CONTRÔLES XYZ ===
        self.create_xyz_controls(left_panel)
        
        # === CONTRÔLES PINCE ===
        self.create_gripper_controls(left_panel)
        
        # === CONTRÔLES TRAJECTOIRE ===
        self.create_trajectory_controls(left_panel)
        
        # === BOUTON QUITTER ===
        quit_frame = tk.Frame(left_panel, bg='#f0f0f0')
        quit_frame.pack(fill=tk.X, padx=5, pady=10)
        
        tk.Button(quit_frame, text="❌ Quitter", 
                 command=self.quit_application,
                 bg='#d32f2f', fg='white', 
                 font=('Arial', 10, 'bold'),
                 height=1).pack(fill=tk.X)
        
        # Initialisation
        self.reset()
        
        # Démarrer le système de clignotement des voyants
        self.start_blinking()
    
    def create_sliders(self, parent):
        """Crée les sliders pour contrôler les servos (version compacte)"""
        servo_names_short = ["Base", "Épaule", "Coude", "Poignet", "Rot.", "Pince"]
        
        for sid in range(1, 7):
            frame = tk.Frame(parent)
            frame.pack(fill=tk.X, pady=1)
            
            label = tk.Label(frame, text=f"M{sid} {servo_names_short[sid-1]}", 
                           width=10, anchor='w', font=('Arial', 8))
            label.pack(side=tk.LEFT)
            
            slider = tk.Scale(
                frame, 
                from_=-150, 
                to=150, 
                orient=tk.HORIZONTAL, 
                length=160,
                resolution=1, 
                command=lambda val, s=sid: self.on_slider(val, s),
                showvalue=0
            )
            slider.set(0)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            
            # Affichage valeur numérique
            value_label = tk.Label(frame, text="0°", width=5, font=('Arial', 8, 'bold'))
            value_label.pack(side=tk.LEFT)
            
            self.sliders[sid] = {'slider': slider, 'label': value_label}
    
    def on_speed_change(self, val):
        """Callback pour le curseur de vitesse"""
        self.current_speed = int(float(val))
        self.speed_label.config(text=str(self.current_speed))
    
    def set_speed(self, speed):
        """Définit une vitesse prédéfinie"""
        self.speed_slider.set(speed)
        self.current_speed = speed
        self.speed_label.config(text=str(speed))
    
    def on_slider(self, val, sid):
        """Callback pour les sliders (utilise vitesse globale)"""
        angle = float(val)
        self.robot.move_servo(sid, angle, speed=self.current_speed)
        self.sliders[sid]['label'].config(text=f"{angle:.0f}°")
        self.update_display()
    
    def update_display(self):
        """Met à jour l'affichage 3D et les coordonnées"""
        angles = [self.sliders[i]['slider'].get() for i in range(1, 6)]
        x, y, z, _ = self.robot.get_end_effector_pose(angles + [0])
        self.label_xyz.config(text=f"X: {x:.1f}  Y: {y:.1f}  Z: {z:.1f}")
        
        # Vérifier l'accessibilité de chaque axe
        self.check_xyz_reachability(x, y, z)
        
        self.update_3d_plot()
    
    def check_xyz_reachability(self, x, y, z):
        """
        Vérifie l'accessibilité de chaque coordonnée et met à jour les voyants
        
        Args:
            x, y, z: Coordonnées à vérifier
        """
        # Paramètres du robot
        L1 = self.robot.L1  # 13 cm
        L2 = self.robot.L2  # 11 cm
        L3 = self.robot.L3  # 11 cm
        L4 = self.robot.L4  # 17 cm
        
        reach_max = L2 + L3  # 22 cm portée maximale
        
        # Position du poignet (avant la pince)
        z_wrist = z + L4
        
        # Distance horizontale
        r = np.sqrt(x**2 + y**2)
        
        # Distance verticale depuis la base
        dz = z_wrist - L1
        
        # Distance totale 3D
        d = np.sqrt(r**2 + dz**2)
        
        # Par défaut, tous les axes sont OK
        x_ok = True
        y_ok = True
        z_ok = True
        
        # Vérifier si la position est globalement atteignable
        if d > reach_max:
            # Position hors de portée - analyser chaque axe
            
            # Méthode: tester si retirer un axe rend la position accessible
            
            # Test X: si on met X=0, est-ce accessible ?
            r_without_x = abs(y)
            d_without_x = np.sqrt(r_without_x**2 + dz**2)
            if d_without_x <= reach_max:
                # Sans X, c'est accessible → X contribue au problème
                x_ok = False
            
            # Test Y: si on met Y=0, est-ce accessible ?
            r_without_y = abs(x)
            d_without_y = np.sqrt(r_without_y**2 + dz**2)
            if d_without_y <= reach_max:
                # Sans Y, c'est accessible → Y contribue au problème
                y_ok = False
            
            # Test Z: si on met Z=0 (neutre), est-ce accessible ?
            z_wrist_neutral = 0 + L4
            dz_neutral = z_wrist_neutral - L1
            d_neutral = np.sqrt(r**2 + dz_neutral**2)
            if d_neutral <= reach_max:
                # Avec Z=0, c'est accessible → Z contribue au problème
                z_ok = False
            
            # Cas spécial: si aucun test ci-dessus n'a détecté de problème,
            # c'est que TOUS les axes contribuent ensemble
            if x_ok and y_ok and z_ok:
                # La combinaison des 3 est trop grande
                # Marquer les axes qui contribuent le plus
                if abs(x) > 5:  # Seuil: contribution significative
                    x_ok = False
                if abs(y) > 5:
                    y_ok = False
                if abs(dz) > 5:
                    z_ok = False
        
        # Vérifications supplémentaires des limites physiques
        
        # Limites en Z (hauteur min/max)
        z_min = L1 - reach_max  # -9 cm
        z_max = L1 + reach_max - L4  # 18 cm
        if z < z_min - 2 or z > z_max + 2:  # Marge de 2cm
            z_ok = False
        
        # Limites en X/Y individuelles (très loin du centre)
        if abs(x) > reach_max - 2:  # 20 cm
            x_ok = False
        if abs(y) > reach_max - 2:  # 20 cm
            y_ok = False
        
        # Mettre à jour les voyants avec clignotement
        self.set_indicator(self.indicator_x, self.circle_x, x_ok, 'x')
        self.set_indicator(self.indicator_y, self.circle_y, y_ok, 'y')
        self.set_indicator(self.indicator_z, self.circle_z, z_ok, 'z')
        
        # Gérer le clignotement
        any_error = not (x_ok and y_ok and z_ok)
        if any_error:
            # Au moins un voyant en erreur → démarrer le clignotement
            self.start_blinking()
        else:
            # Tout est OK → arrêter le clignotement
            self.stop_blinking()
    
    def set_indicator(self, canvas, circle, is_ok, axis):
        """
        Met à jour un voyant individuel avec clignotement si hors limites
        
        Args:
            canvas: Canvas tkinter
            circle: ID du cercle
            is_ok: True si OK (vert), False si problème (rouge clignotant)
            axis: 'x', 'y' ou 'z'
        """
        # Sauvegarder l'état de l'indicateur
        self.indicators_status[axis] = is_ok
        
        if is_ok:
            # Vert fixe : position accessible
            canvas.itemconfig(circle, fill='#4CAF50', outline='#2E7D32')
        else:
            # Rouge clignotant : position inaccessible
            if self.blink_state:
                # État allumé : rouge vif
                canvas.itemconfig(circle, fill='#F44336', outline='#C62828')
            else:
                # État éteint : rouge sombre
                canvas.itemconfig(circle, fill='#B71C1C', outline='#7F0000')
    
    def blink_indicators(self):
        """
        Fait clignoter les voyants rouges (appelé périodiquement)
        """
        # Inverser l'état du clignotement
        self.blink_state = not self.blink_state
        
        # Mettre à jour uniquement les voyants qui sont en erreur
        if not self.indicators_status['x']:
            self.set_indicator(self.indicator_x, self.circle_x, False, 'x')
        
        if not self.indicators_status['y']:
            self.set_indicator(self.indicator_y, self.circle_y, False, 'y')
        
        if not self.indicators_status['z']:
            self.set_indicator(self.indicator_z, self.circle_z, False, 'z')
        
        # Programmer le prochain clignotement (500ms = 2 fois par seconde)
        self.blink_timer = self.root.after(500, self.blink_indicators)
    
    def start_blinking(self):
        """Démarre le clignotement des voyants"""
        if self.blink_timer is None:
            self.blink_indicators()
    
    def stop_blinking(self):
        """Arrête le clignotement des voyants"""
        if self.blink_timer is not None:
            self.root.after_cancel(self.blink_timer)
            self.blink_timer = None
            self.blink_state = False
    
    def update_3d_plot(self):
        """Met à jour la visualisation 3D du robot"""
        self.ax.clear()
        
        # Configuration axes
        self.ax.set_xlim(-25, 25)
        self.ax.set_ylim(-25, 25)
        self.ax.set_zlim(0, 60)
        self.ax.set_xlabel("X (cm)")
        self.ax.set_ylabel("Y (cm)")
        self.ax.set_zlabel("Z (cm)")
        self.ax.view_init(elev=25, azim=135)
        
        # Récupération des angles (tous les 6 axes)
        angles = [self.sliders[i]['slider'].get() for i in range(1, 7)]
        
        # Calcul des positions avec la VRAIE cinématique directe
        transforms = self.robot.forward_kinematics_full(angles)
        
        # Extraction des positions de chaque articulation
        points = []
        for T in transforms:
            pos = T[0:3, 3]
            points.append(pos)
        
        # Tracé des segments (cylindres) - 4 segments maintenant
        colors = ["#0077be", "#00a86b", "#ffa500", "#d62728"]
        
        for i in range(4):  # 4 segments : Base→M1, M1→M2, M2→M3, M3→M4 (poignet)
            p1, p2 = points[i], points[i + 1]
            self.draw_cylinder(p1, p2, radius=1.5, color=colors[i])
        
        # Position de la pince = position M4 = M5 = M6 (poignet)
        gripper_center = transforms[4][0:3, 3]  # M4, M5, M6 au même endroit
        
        # Tracé de la pince avec rotation correcte (T5 avec offset +90° en M5)
        T_gripper = transforms[5]  # T5 (avec rotation M5 qui pointe vers le haut)
        
        # === NOUVELLE PINCE RÉALISTE ===
        # La pince commence au poignet et fait 10 cm vers le haut
        
        # Matrice d'orientation de la pince
        R_gripper = T_gripper[0:3, 0:3]
        
        # Axes locaux de la pince (avec rotation de +90° en M5)
        gripper_x = R_gripper[:, 0]  # Axe X local (direction principale - vers le haut)
        gripper_y = R_gripper[:, 1]  # Axe Y local (ouverture)
        gripper_z = R_gripper[:, 2]  # Axe Z local (rotation pince)
        
        # Calcul ouverture pince (angle moteur 6)
        grip_angle = self.sliders[6]['slider'].get()
        grip_opening = self.map_gripper_opening(grip_angle)
        
        # 1. PIÈCE FIXE (bloc de liaison au poignet)
        # Position : commence au centre du poignet
        base_length = 3.0  # 3 cm
        base_width = 4.0
        base_height = 3.0
        
        # La base commence au poignet et s'étend selon gripper_x (vers le haut)
        base_center = gripper_center + gripper_x * (base_length / 2)
        self.draw_box(base_center, base_width, base_height, base_length, 
                     R_gripper, "#555555")  # Gris foncé
        
        # 2. MORS RECTANGULAIRES (2 doigts)
        # Position : après la base, s'étendent vers le haut (gripper_x)
        jaw_start_pos = gripper_center + gripper_x * base_length
        
        jaw_length = 5.0   # 5 cm de long (réduit pour axe bleu)
        jaw_width = 1.5    # 1.5 cm de large
        jaw_height = 3.5   # 3.5 cm de haut (augmenté pour axe vert)
        
        # Écartement des mors selon l'ouverture
        # grip_opening est en cm (maintenant 1.0 à 5.0 cm)
        jaw_offset = grip_opening / 2  # Distance du centre aux mors
        
        # Mors 1 (à droite selon Y local)
        jaw1_center = jaw_start_pos + gripper_y * jaw_offset + gripper_x * (jaw_length / 2)
        self.draw_box(jaw1_center, jaw_width, jaw_height, jaw_length,
                     R_gripper, "#888888")  # Gris
        
        # Mors 2 (à gauche selon Y local)
        jaw2_center = jaw_start_pos - gripper_y * jaw_offset + gripper_x * (jaw_length / 2)
        self.draw_box(jaw2_center, jaw_width, jaw_height, jaw_length,
                     R_gripper, "#888888")  # Gris
        
        # Articulations (sphères) - seulement les 4 premières
        for i in range(5):  # T0 à T4
            self.ax.scatter(*points[i], color="black", s=30, marker='o')
        
        # Repère de base (monde)
        origin = np.array([0, 0, 0])
        self.ax.quiver(*origin, 5, 0, 0, color="red", arrow_length_ratio=0.2, linewidth=2)
        self.ax.quiver(*origin, 0, 5, 0, color="green", arrow_length_ratio=0.2, linewidth=2)
        self.ax.quiver(*origin, 0, 0, 5, color="blue", arrow_length_ratio=0.2, linewidth=2)
        
        # Repère de la pince (à l'extrémité des mors)
        scale = 4  # Taille des flèches
        gripper_tip = gripper_center + gripper_x * (base_length + jaw_length)
        self.ax.quiver(*gripper_tip, *(gripper_x * scale), 
                      color="red", arrow_length_ratio=0.3, linewidth=2, alpha=0.8, label="X pince")
        self.ax.quiver(*gripper_tip, *(gripper_y * scale), 
                      color="green", arrow_length_ratio=0.3, linewidth=2, alpha=0.8, label="Y pince")
        self.ax.quiver(*gripper_tip, *(gripper_z * scale), 
                      color="blue", arrow_length_ratio=0.3, linewidth=2, alpha=0.8, label="Z pince")
        
        # Grille
        self.ax.grid(True, alpha=0.3)
        
        self.canvas.draw()
    
    def draw_cylinder(self, p1, p2, radius, color):
        """Dessine un cylindre entre deux points"""
        p1, p2 = np.array(p1), np.array(p2)
        v = p2 - p1
        length = np.linalg.norm(v)
        
        if length < 1e-6:
            return
        
        v = v / length
        
        # Cylindre de base le long de Z
        theta = np.linspace(0, 2 * np.pi, 16)
        z = np.linspace(0, length, 2)
        theta_grid, z_grid = np.meshgrid(theta, z)
        x_grid = radius * np.cos(theta_grid)
        y_grid = radius * np.sin(theta_grid)
        
        # Matrice de rotation pour aligner Z sur v
        z_axis = np.array([0, 0, 1])
        
        if np.allclose(v, z_axis):
            R = np.eye(3)
        elif np.allclose(v, -z_axis):
            R = -np.eye(3)
            R[2, 2] = 1
        else:
            # Rodrigues formula
            k = np.cross(z_axis, v)
            k = k / np.linalg.norm(k)
            K = np.array([
                [0, -k[2], k[1]],
                [k[2], 0, -k[0]],
                [-k[1], k[0], 0]
            ])
            angle = np.arccos(np.dot(z_axis, v))
            R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        
        # Application de la rotation et translation
        xyz = np.array([x_grid.flatten(), y_grid.flatten(), z_grid.flatten()])
        rotated = R @ xyz
        
        x_final = rotated[0, :].reshape(x_grid.shape) + p1[0]
        y_final = rotated[1, :].reshape(y_grid.shape) + p1[1]
        z_final = rotated[2, :].reshape(z_grid.shape) + p1[2]
        
        self.ax.plot_surface(x_final, y_final, z_final, 
                            color=color, alpha=0.8, shade=True)
    
    def draw_box(self, center, width, height, depth, orientation_matrix, color):
        """
        Dessine un parallélépipède (boîte rectangulaire)
        
        Args:
            center: Position du centre [x, y, z]
            width: Largeur (selon axe X local)
            height: Hauteur (selon axe Y local)
            depth: Profondeur (selon axe Z local)
            orientation_matrix: Matrice 3x3 d'orientation
            color: Couleur
        """
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        
        # Définir les coins du parallélépipède dans le repère local
        dx, dy, dz = width/2, height/2, depth/2
        corners_local = np.array([
            [-dx, -dy, -dz],  # 0
            [ dx, -dy, -dz],  # 1
            [ dx,  dy, -dz],  # 2
            [-dx,  dy, -dz],  # 3
            [-dx, -dy,  dz],  # 4
            [ dx, -dy,  dz],  # 5
            [ dx,  dy,  dz],  # 6
            [-dx,  dy,  dz],  # 7
        ])
        
        # Rotation et translation
        corners = (orientation_matrix @ corners_local.T).T + center
        
        # Définir les 6 faces (chaque face = 4 coins)
        faces = [
            [0, 1, 2, 3],  # Face Z-
            [4, 5, 6, 7],  # Face Z+
            [0, 1, 5, 4],  # Face Y-
            [2, 3, 7, 6],  # Face Y+
            [0, 3, 7, 4],  # Face X-
            [1, 2, 6, 5],  # Face X+
        ]
        
        # Dessiner chaque face
        face_vertices = []
        for face in faces:
            face_vertices.append([corners[i] for i in face])
        
        poly = Poly3DCollection(face_vertices, alpha=0.8, facecolor=color, 
                               edgecolor='black', linewidth=0.5)
        self.ax.add_collection3d(poly)
    
    def map_gripper_opening(self, angle):
        """Convertit l'angle de la pince en ouverture (écartement entre mors)"""
        # Mapping: -90° → ouvert (5.0 cm), +150° → fermé (1.0 cm)
        grip_min, grip_max = -90, 150
        open_dist, closed_dist = 5.0, 1.0  # Écartement agrandi
        
        angle_clamped = np.clip(angle, grip_min, grip_max)
        ratio = (angle_clamped - grip_min) / (grip_max - grip_min)
        
        return open_dist + (closed_dist - open_dist) * ratio
    
    def create_xyz_controls(self, parent):
        """Crée les contrôles pour les coordonnées cartésiennes (version compacte)"""
        frame = tk.LabelFrame(parent, text="📍 XYZ", 
                             font=('Arial', 10, 'bold'), padx=5, pady=5)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.xyz_entries = {}
        default_values = {'X': '13', 'Y': '0', 'Z': '0'}
        
        coords_frame = tk.Frame(frame)
        coords_frame.pack(fill=tk.X, pady=2)
        
        for label, default in default_values.items():
            sub = tk.Frame(coords_frame)
            sub.pack(side=tk.LEFT, expand=True, padx=2)
            tk.Label(sub, text=f"{label}:", font=('Arial', 8)).pack()
            entry = tk.Entry(sub, width=5, font=('Arial', 9), justify='center')
            entry.insert(0, default)
            entry.pack()
            self.xyz_entries[label] = entry
        
        tk.Button(frame, text="🎯 Atteindre", 
                 command=self.go_to_xyz, bg='#4CAF50', fg='white',
                 font=('Arial', 8, 'bold')).pack(fill=tk.X, pady=2)
    
    def go_to_xyz(self):
        """Déplace le robot vers des coordonnées cartésiennes (utilise vitesse globale)"""
        try:
            # Lecture des valeurs
            x = float(self.xyz_entries['X'].get())
            y = float(self.xyz_entries['Y'].get())
            z = float(self.xyz_entries['Z'].get())
            
            print(f"🎯 Déplacement vers: X={x}, Y={y}, Z={z}")
            
            angles = self.robot.inverse_kinematics(x, y, z)
            
            if angles:
                print(f"✅ Angles calculés: θ1={angles[0]:.1f}°, θ2={angles[1]:.1f}°, θ3={angles[2]:.1f}°")
                
                t1, t2, t3 = angles
                t4 = -(t2 + t3) - 173  # Offset qui fonctionnait
                
                full_angles = [t1, t2, t3, t4, 0, self.sliders[6]['slider'].get()]
                
                print(f"▶️  Mouvement direct (vitesse: {self.current_speed})...")
                
                # Envoyer TOUTES les commandes avec la vitesse globale
                for i, angle in enumerate(full_angles):
                    self.robot.move_servo(i + 1, angle, self.current_speed)
                
                # Attendre que TOUS les servos aient fini
                self.robot.wait_for_all_servos()
                
                # Mise à jour interface
                for i, angle in enumerate(full_angles):
                    self.sliders[i + 1]['slider'].set(angle)
                    self.sliders[i + 1]['label'].config(text=f"{angle:.0f}°")
                
                self.update_display()
                
                print(f"✅ Position atteinte: ({x:.1f}, {y:.1f}, {z:.1f})")
            else:
                # Position hors de portée : mettre à jour les voyants X, Y, Z
                self.check_xyz_reachability(x, y, z)
                print(f"🚫 Position hors de portée")
                
        except ValueError as e:
            print(f"❌ Coordonnées invalides: {e}")
        except Exception as e:
            print(f"❌ Erreur: {e}")
            import traceback
            traceback.print_exc()
    
    def create_gripper_controls(self, parent):
        """Crée les contrôles pour la pince (version compacte)"""
        frame = tk.LabelFrame(parent, text="✋ Pince", 
                             font=('Arial', 10, 'bold'), padx=5, pady=5)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        angles = [("🔓", -90), ("🔒", 150)]
        
        btn_frame = tk.Frame(frame)
        btn_frame.pack(fill=tk.X)
        
        for text, angle in angles:
            tk.Button(btn_frame, text=text, width=8, font=('Arial', 9),
                command=lambda a=angle: self.set_gripper(a)).pack(
                    side=tk.LEFT, expand=True, fill=tk.X, padx=1)
    
    def set_gripper(self, angle):
        """Règle l'ouverture de la pince (utilise vitesse globale)"""
        self.sliders[6]['slider'].set(angle)
        self.robot.move_servo(6, angle, speed=self.current_speed)
        self.sliders[6]['label'].config(text=f"{angle}°")
        self.update_display()
    
    def create_trajectory_controls(self, parent):
        """Crée les contrôles pour les trajectoires (version compacte)"""
        frame = tk.LabelFrame(parent, text="📹 Trajectoires", 
                             font=('Arial', 10, 'bold'), padx=5, pady=5)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Ligne 1
        btn1 = tk.Frame(frame)
        btn1.pack(fill=tk.X, pady=1)
        tk.Button(btn1, text="📍 Enreg.", command=self.record_position,
                 bg='#2196F3', fg='white', font=('Arial', 8, 'bold')).pack(
                     side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        tk.Button(btn1, text="🗑️ Effacer", command=self.clear_recorded,
                 bg='#f44336', fg='white', font=('Arial', 8, 'bold')).pack(
                     side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        
        # Ligne 2
        btn2 = tk.Frame(frame)
        btn2.pack(fill=tk.X, pady=1)
        tk.Button(btn2, text="▶️ Jouer", command=self.play_trajectory_smooth,
                 bg='#4CAF50', fg='white', font=('Arial', 8, 'bold')).pack(
                     side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        tk.Button(btn2, text="💾 Sauver", command=self.save_trajectory,
                 bg='#FF9800', fg='white', font=('Arial', 8, 'bold')).pack(
                     side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        tk.Button(btn2, text="📂 Charger", command=self.load_trajectory,
                 bg='#9C27B0', fg='white', font=('Arial', 8, 'bold')).pack(
                     side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        
        # Compteur
        self.position_label = tk.Label(frame, text="0 pos", 
                                      fg='#666', font=('Arial', 8))
        self.position_label.pack(pady=2)
    
    def record_position(self):
        """Enregistre la position actuelle avec la vitesse"""
        angles = [self.sliders[i]['slider'].get() for i in range(1, 7)]
        speed = self.current_speed
        # Format : [angle1, angle2, angle3, angle4, angle5, angle6, speed]
        position_with_speed = angles + [speed]
        self.recorded_positions.append(position_with_speed)
        self.position_label.config(text=f"{len(self.recorded_positions)} pos")
        print(f"✅ Position {len(self.recorded_positions)} enregistrée (vitesse: {speed})")
    
    def play_trajectory_smooth(self):
        """Rejoue la trajectoire avec les vitesses enregistrées"""
        if len(self.recorded_positions) < 2:
            print("❌ Au moins 2 positions nécessaires")
            return
        
        print(f"▶️  Lecture de {len(self.recorded_positions)} positions...")
        
        # Jouer chaque waypoint avec sa vitesse enregistrée
        for i, position_data in enumerate(self.recorded_positions):
            # Format : [angle1, ..., angle6, speed] (7 valeurs)
            # ou ancien format : [angle1, ..., angle6] (6 valeurs)
            if len(position_data) == 7:
                # Nouveau format avec vitesse
                target_angles = position_data[:6]
                speed = int(position_data[6])
            else:
                # Ancien format sans vitesse (compatibilité)
                target_angles = position_data
                speed = self.current_speed
            
            print(f"   → Waypoint {i+1}/{len(self.recorded_positions)} (vitesse: {speed})")
            
            # Envoyer toutes les commandes avec la vitesse enregistrée
            for j, angle in enumerate(target_angles):
                self.robot.move_servo(j + 1, angle, speed)
            
            # ATTENDRE que TOUS les servos aient vraiment fini
            self.robot.wait_for_all_servos()
            
            # Mise à jour interface
            for j, angle in enumerate(target_angles):
                self.sliders[j + 1]['slider'].set(angle)
                self.sliders[j + 1]['label'].config(text=f"{angle:.0f}°")
            
            self.update_display()
            self.root.update()
            
            # Petite pause entre waypoints
            time.sleep(0.2)
        
        print("✅ Trajectoire terminée")
    
    def execute_trajectory(self, trajectory, speed=150):
        """
        Exécute une trajectoire interpolée
        
        Args:
            trajectory: np.array de positions
            speed: vitesse unique ou array de vitesses
        """
        for i, position in enumerate(trajectory):
            # Vitesse pour cette étape
            if isinstance(speed, (list, np.ndarray)):
                current_speed = int(speed[i])
            else:
                current_speed = speed
            
            # Envoi des commandes
            for j, angle in enumerate(position):
                self.robot.move_servo(j + 1, angle, current_speed)
                self.sliders[j + 1]['slider'].set(angle)
                self.sliders[j + 1]['label'].config(text=f"{angle:.0f}°")
            
            # Mise à jour affichage (tous les N points pour performances)
            if i % 5 == 0:
                self.update_display()
                self.root.update()
            
            # Petit délai pour laisser les servos bouger
            time.sleep(0.02)
        
        # Attente fin de mouvement
        self.robot.wait_for_all_servos()
        self.update_display()
    
    def execute_trajectory_for_dynamixel(self, trajectory, speeds):
        """
        Exécute une trajectoire OPTIMISÉE pour servos Dynamixel AX-12
        
        Prend en compte le temps de réponse réel des servos (~50-80ms)
        
        Args:
            trajectory: np.array de positions (peu de points : 10-20)
            speeds: array de vitesses pour chaque point
        """
        print(f"▶️  Exécution de {len(trajectory)} points (optimisé Dynamixel)...")
        
        for i, position in enumerate(trajectory):
            # Vitesse variable selon le profil
            current_speed = int(speeds[i])
            
            # Envoi simultané à tous les servos
            for j, angle in enumerate(position):
                self.robot.move_servo(j + 1, angle, current_speed)
            
            # Mise à jour interface
            for j, angle in enumerate(position):
                self.sliders[j + 1]['slider'].set(angle)
                self.sliders[j + 1]['label'].config(text=f"{angle:.0f}°")
            
            # Mise à jour 3D
            self.update_display()
            self.root.update()
            
            # 🔥 CRITIQUE : Laisser le temps aux servos de bouger !
            # Les AX-12 ont besoin de 50-80ms pour un petit déplacement
            time.sleep(0.08)  # 80ms entre chaque point
        
        # Attente finale
        self.robot.wait_for_all_servos()
        self.update_display()
        print("✅ Mouvement terminé")
    
    def execute_trajectory_smooth(self, trajectory, speeds):
        """
        Exécute une trajectoire de manière ULTRA-FLUIDE (optimisée)
        
        Args:
            trajectory: np.array de positions
            speeds: array de vitesses pour chaque point
        """
        print(f"▶️  Exécution fluide de {len(trajectory)} points...")
        
        for i, position in enumerate(trajectory):
            # Vitesse variable selon le profil
            current_speed = int(speeds[i])
            
            # Envoi des commandes aux servos
            for j, angle in enumerate(position):
                self.robot.move_servo(j + 1, angle, current_speed)
            
            # 🔥 OPTIMISATION 1 : Mise à jour sliders moins fréquente
            if i % 10 == 0:  # Tous les 10 points au lieu de chaque point
                for j, angle in enumerate(position):
                    self.sliders[j + 1]['slider'].set(angle)
                    self.sliders[j + 1]['label'].config(text=f"{angle:.0f}°")
            
            # 🔥 OPTIMISATION 2 : Mise à jour 3D encore moins fréquente
            if i % 20 == 0:  # Tous les 20 points
                self.update_display()
                self.root.update()
            
            # 🔥 OPTIMISATION 3 : Délai minimal entre points
            time.sleep(0.005)  # 5ms au lieu de 20ms !
        
        # Mise à jour finale
        final_position = trajectory[-1]
        for j, angle in enumerate(final_position):
            self.sliders[j + 1]['slider'].set(angle)
            self.sliders[j + 1]['label'].config(text=f"{angle:.0f}°")
        
        self.robot.wait_for_all_servos()
        self.update_display()
        print("✅ Mouvement terminé")
    
    def save_trajectory(self):
        """Sauvegarde la trajectoire avec vitesses dans un fichier CSV"""
        if not self.recorded_positions:
            print("❌ Aucune position à sauvegarder")
            return
        
        name = simpledialog.askstring("Nom", "Nom de la trajectoire :")
        if name:
            filename = f"{name}.csv"
            with open(filename, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                # Nouvelle entête avec colonne Speed
                writer.writerow(['Servo1', 'Servo2', 'Servo3', 
                               'Servo4', 'Servo5', 'Servo6', 'Speed'])
                for pos_data in self.recorded_positions:
                    # Format : [angle1, ..., angle6, speed]
                    if len(pos_data) == 7:
                        # Nouveau format
                        writer.writerow([f"{val:.2f}" for val in pos_data])
                    else:
                        # Ancien format (compatibilité) - ajouter vitesse par défaut
                        row = [f"{angle:.2f}" for angle in pos_data] + [f"{self.current_speed:.2f}"]
                        writer.writerow(row)
            print(f"💾 Trajectoire sauvegardée dans {filename} (avec vitesses)")
    
    def load_trajectory(self):
        """Charge une trajectoire avec vitesses depuis un fichier CSV"""
        filename = filedialog.askopenfilename(
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if filename:
            self.recorded_positions.clear()
            with open(filename, newline="") as csvfile:
                reader = csv.reader(csvfile)
                header = next(reader, None)  # Lire l'entête
                
                # Vérifier si le fichier contient la colonne Speed
                has_speed = header and 'Speed' in header
                
                for row in reader:
                    if has_speed and len(row) == 7:
                        # Nouveau format : [angle1, ..., angle6, speed]
                        position_data = [float(val) for val in row]
                    elif len(row) == 6:
                        # Ancien format : [angle1, ..., angle6] - ajouter vitesse par défaut
                        angles = [float(angle) for angle in row]
                        position_data = angles + [200.0]  # Vitesse par défaut
                    else:
                        print(f"⚠️  Ligne ignorée (format incorrect): {row}")
                        continue
                    
                    self.recorded_positions.append(position_data)
            
            self.position_label.config(text=f"{len(self.recorded_positions)} pos")
            
            if has_speed:
                print(f"📂 {len(self.recorded_positions)} positions chargées (avec vitesses)")
            else:
                print(f"📂 {len(self.recorded_positions)} positions chargées (vitesse par défaut: 200)")

    
    def clear_recorded(self):
        """Efface les positions enregistrées"""
        self.recorded_positions.clear()
        self.position_label.config(text="0 pos")
        print("🗑️  Positions effacées")
    
    def quit_application(self):
        """Ferme proprement l'application"""
        if messagebox.askokcancel("Quitter", "Voulez-vous vraiment quitter ?"):
            print("👋 Fermeture de l'application...")
            # Arrêter le clignotement des voyants
            self.stop_blinking()
            self.root.quit()
            self.root.destroy()
    
    def reset(self):
        """Réinitialise tous les servos à 0°"""
        for sid in range(1, 7):
            self.sliders[sid]['slider'].set(0)
            self.sliders[sid]['label'].config(text="0°")
            self.robot.move_servo(sid, 0, speed=100)
        
        self.robot.wait_for_all_servos()
        self.update_display()
        print("🔄 Robot réinitialisé")
    
    def run(self):
        """Lance l'interface graphique"""
        self.root.mainloop()


# ============================================================================
# FONCTION DE SÉLECTION DU PORT COM
# ============================================================================

def select_com_port():
    """Affiche une boîte de dialogue pour sélectionner le port COM"""
    import serial.tools.list_ports
    
    # Créer fenêtre temporaire
    root = tk.Tk()
    root.withdraw()  # Cacher la fenêtre principale
    
    # Lister les ports COM disponibles
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    
    if not available_ports:
        # Aucun port détecté, proposer la saisie manuelle
        messagebox.showwarning(
            "Aucun port détecté",
            "Aucun port série détecté.\nVous pouvez saisir manuellement le port COM."
        )
        port = simpledialog.askstring(
            "Port COM",
            "Entrez le nom du port COM (ex: COM6 ou /dev/ttyUSB0):",
            initialvalue="COM6"
        )
        root.destroy()
        return port if port else "COM6"
    
    # Créer fenêtre de sélection
    root.deiconify()  # Montrer la fenêtre
    root.title("Sélection du Port COM")
    root.geometry("400x300")
    root.resizable(False, False)
    
    # Centrer la fenêtre
    root.update_idletasks()
    width = root.winfo_width()
    height = root.winfo_height()
    x = (root.winfo_screenwidth() // 2) - (width // 2)
    y = (root.winfo_screenheight() // 2) - (height // 2)
    root.geometry(f'{width}x{height}+{x}+{y}')
    
    selected_port = tk.StringVar(value=available_ports[0])
    
    # Titre
    tk.Label(
        root,
        text="🔌 Sélection du Port COM",
        font=("Arial", 14, "bold"),
        pady=10
    ).pack()
    
    # Instructions
    tk.Label(
        root,
        text="Sélectionnez le port série pour le robot:",
        font=("Arial", 10),
        pady=5
    ).pack()
    
    # Frame pour la liste
    list_frame = tk.Frame(root, padx=20, pady=10)
    list_frame.pack(fill=tk.BOTH, expand=True)
    
    # Liste des ports avec scrollbar
    scrollbar = tk.Scrollbar(list_frame)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    
    listbox = tk.Listbox(
        list_frame,
        font=("Courier", 10),
        yscrollcommand=scrollbar.set,
        height=8
    )
    listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.config(command=listbox.yview)
    
    # Remplir la liste
    for i, port in enumerate(ports):
        port_info = f"{port.device} - {port.description}"
        listbox.insert(tk.END, port_info)
        if i == 0:
            listbox.selection_set(0)
    
    def on_select():
        selection = listbox.curselection()
        if selection:
            selected_port.set(available_ports[selection[0]])
        root.quit()
    
    def on_manual():
        port = simpledialog.askstring(
            "Saisie manuelle",
            "Entrez le nom du port COM:",
            initialvalue=selected_port.get()
        )
        if port:
            selected_port.set(port)
        root.quit()
    
    # Boutons
    button_frame = tk.Frame(root, pady=10)
    button_frame.pack()
    
    tk.Button(
        button_frame,
        text="✓ Sélectionner",
        command=on_select,
        bg="#4CAF50",
        fg="white",
        font=("Arial", 10, "bold"),
        width=15,
        pady=5
    ).pack(side=tk.LEFT, padx=5)
    
    tk.Button(
        button_frame,
        text="✏️ Saisie manuelle",
        command=on_manual,
        bg="#2196F3",
        fg="white",
        font=("Arial", 10, "bold"),
        width=15,
        pady=5
    ).pack(side=tk.LEFT, padx=5)
    
    # Lancer la fenêtre
    root.mainloop()
    root.destroy()
    
    return selected_port.get()


# ============================================================================
# PROGRAMME PRINCIPAL
# ============================================================================

if __name__ == "__main__":
    try:
        # Sélection du port COM
        print("🔌 Sélection du port COM...")
        port_name = select_com_port()
        
        if not port_name:
            print("❌ Aucun port sélectionné. Arrêt du programme.")
            exit(1)
        
        print(f"✓ Port sélectionné: {port_name}")
        
        # Initialisation du robot
        print("🤖 Initialisation du robot...")
        robot = Robot6Axes(port_name)
        
        # Lancement de l'interface
        print("🖥️  Lancement de l'interface...")
        gui = RobotGUI(robot)
        gui.run()
        
    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()
