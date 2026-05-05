import numpy as np
import time
import struct

class LegKinematics:
    def __init__(self):
        # Longitudes reales de tu diseño
        self.L1, self.L2, self.L3 = 86.2, 150.0, 150.0

    def get_joint_angles(self, coord):
        x, y, z = coord
        r_yz = np.sqrt(y**2 + z**2)
        alpha = np.arccos(np.clip(y/r_yz, -1, 1))
        th0 = np.arccos(np.clip(self.L1/r_yz, -1, 1)) - alpha
        r_p = np.sqrt(max(0, r_yz**2 - self.L1**2))
        d_sq = x**2 + r_p**2
        d = np.sqrt(d_sq)
        cos_k = (self.L2**2 + self.L3**2 - d_sq) / (2 * self.L2 * self.L3)
        knee_angle = np.arccos(np.clip(cos_k, -1, 1))
        
        phi1 = np.arctan2(r_p, x)
        phi2 = np.arccos(np.clip((self.L2**2 + d_sq - self.L3**2) / (2 * self.L2 * d), -1, 1))

        th1 = phi1 - phi2
        th2 = np.pi - knee_angle
        #th2 = th1 + th2 #Por si no hay cinemática relativa
        
        # Salida en radianes
        return np.array([th0, th1, th2])

def get_full_cycle(steps=20): # Pocos pasos para revisar la consola fácil
    z_ground, y_offset, step_height, step_length = -180.0, 130.0, 40.0, 60.0
    p0 = np.array([-step_length, y_offset, z_ground])
    p1 = np.array([-step_length, y_offset, z_ground + step_height])
    p2 = np.array([step_length,  y_offset, z_ground + step_height])
    p3 = np.array([step_length,  y_offset, z_ground])

    t = np.linspace(0, 1, steps//2)
    vuelo = np.array([(1-ti)**3*p0 + 3*(1-ti)**2*ti*p1 + 3*(1-ti)*ti**2*p2 + ti**3*p3 for ti in t])
    apoyo = np.linspace(p3, p0, steps//2)
    return np.vstack((apoyo, vuelo))

# --- SIMULACIÓN DE HARDWARE ---
kin = LegKinematics()
trayectoria = get_full_cycle()

# Offsets que mencionaste
off1, off2, off3 = 135.0, 120.0, 180.0

print(f"{'FASE':<10} | {'X':<6} {'Z':<6} | {'IK_Q1':<7} {'IK_Q2':<7} {'IK_Q3':<7} | {'SERVO1':<7} {'SERVO2':<7} {'SERVO3':<7}")
print("-" * 95)

for i, punto in enumerate(trayectoria):
    # 1. Obtener ángulos de la cinemática (en radianes)
    q_rad = kin.get_joint_angles(punto)
    q_deg = np.degrees(q_rad) # Convertir a grados
    
    # 2. Aplicar tus reglas de hardware:
    # Q1: offset + IK (suma)
    s1 = off1 + q_deg[0]
    # Q2: offset - IK (resta)
    s2 = off2 - q_deg[1]
    # Q3: offset - IK (resta)
    s3 = off3 - q_deg[2]
    
    fase = "APOYO" if i < len(trayectoria)//2 else "VUELO"
    
    # Imprimir valores para inspección
    print(f"{fase:<10} | {punto[0]:>6.1f} {punto[2]:>6.1f} | {q_deg[0]:>7.1f} {q_deg[1]:>7.1f} {q_deg[2]:>7.1f} | {s1:>7.1f} {s2:>7.1f} {s3:>7.1f}")
    
    # Simular el empaquetado que irá al ESP32 (Prueba de bytes)
    # payload = struct.pack('<fff', q_deg[0], q_deg[1], q_deg[2])
    
    time.sleep(0.05)

print("\n[OK] Inspección completada. Revisa que los valores SERVO estén entre 0 y 180.")