import numpy as np
import matplotlib.pyplot as plt

def get_full_cycle_final(steps=40):
    # Parámetros base
    z_ground, y_offset, step_height, step_length = -180.0, 130.0, 45.0, 60.0

    # Proporción 3/4 Stance (75%) y 1/4 Swing (25%)
    steps_stance = int(steps * 0.75)
    steps_swing = steps - steps_stance

    # --- 1. STANCE SENOIDAL (Apoyo) ---
    t_s = np.linspace(0, np.pi, steps_stance)
    # Movimiento en X suavizado (Coseno) para evitar tirones
    x_s = step_length - (2 * step_length * (1 - np.cos(t_s)) / 2)
    apoyo = np.column_stack((x_s, np.full_like(x_s, z_ground)))

    # --- 2. SWING BÉZIER G5 (Vuelo) ---
    # Puntos de control para verticalidad y suavidad total (Vz=0, Az=0)
    p = [
        np.array([-step_length, z_ground]),          # P0 (Inicio)
        np.array([-step_length, z_ground]),          # P1 (Suavidad)
        np.array([-45.0, z_ground + step_height*1.2]), # P2 (Verticalidad)
        np.array([ 45.0, z_ground + step_height*1.2]), # P3 (Verticalidad)
        np.array([ step_length, z_ground]),          # P4 (Suavidad)
        np.array([ step_length, z_ground])           # P5 (Fin)
    ]

    t_v = np.linspace(0, 1, steps_swing)
    vuelo = np.array([(1-ti)**5*p[0] + 5*(1-ti)**4*ti*p[1] + 10*(1-ti)**3*ti**2*p[2] +
                      10*(1-ti)**2*ti**3*p[3] + 5*(1-ti)*ti**4*p[4] + ti**5*p[5] for ti in t_v])

    return apoyo, vuelo

# Generar y Graficar
apoyo, vuelo = get_full_cycle_final(60) # Usamos 60 para que se vea bien la densidad

plt.figure(figsize=(12, 6))

# Dibujar Apoyo (Stance)
plt.scatter(apoyo[:,0], apoyo[:,1], c='navy', s=30, label='Stance Senoidal (75% del tiempo)', zorder=3)
# Dibujar Vuelo (Swing)
plt.scatter(vuelo[:,0], vuelo[:,1], c='dodgerblue', s=45, label='Swing Bézier G5 (25% del tiempo)', edgecolors='black', zorder=4)

# Líneas de trayectoria
plt.plot(np.vstack((apoyo, vuelo[0]))[:,0], np.vstack((apoyo, vuelo[0]))[:,1], 'k--', alpha=0.2)
plt.plot(vuelo[:,0], vuelo[:,1], 'b-', alpha=0.2)

plt.axhline(y=-180, color='red', linestyle='-', alpha=0.3, label='Nivel del Suelo')
plt.title("Perfil de movimiento de una pata basado en Bezier G5", fontsize=14)
plt.xlabel("Posición X (mm)")
plt.ylabel("Altura Z (mm)")
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis('equal')

plt.show()