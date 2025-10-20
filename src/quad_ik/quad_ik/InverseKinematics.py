mport numpy as np
from ntpath import join
# # ==========================
# # Función de cinemática inversa antigua
# # ==========================
# def mgi_quadruped(x, y, z):
#     #Longitud de cada eslabón(medir)
#     L1 = 0.102
#     L2 = 0.15
#     L3 = 0.15

#     # q1 solución negativa
#     q1 = np.arctan2(-x, y) - np.arctan2(np.sqrt(x**2 + y**2 - L1**2), L1)

#     # Variables auxiliares
#     Z1 = -x*np.cos(q1) - y*np.sin(q1)
#     Z2 = z

#     A = Z1**2 + Z2**2 - L2**2 - L3**2
#     B = 2 * L2 * L3
#     C = A / B

#     # q3 solución positiva
#     q3 = np.arctan2(-np.sqrt(1 - C**2), C)

#     # Calcular q2
#     B1 = L2 + L3*C
#     B2 = L3 * np.sin(q3)
#     S1 = B1*Z2 - B2*Z1
#     S2 = B1**2 + B2**2
#     S = S1 / S2
#     T1 = B1*Z1 + B2*Z2
#     T = T1 / S2
#     q2 = np.arctan2(S, T)

#     return np.array([q1, q2, q3])
# ==========================
# Estados de las patas
# ==========================
class LegStates():
    prev_coord = None #Coordenada cartesiana previa
    now_coord = None #Coordenada cartesiana actual
    prev_angles = None #Coordenada articular previa
    now_angles = None #Coordenada articular actual
    in_singularity = None #Estado de singularidad

class Legs():
    FR = LegStates() #Pata Frontal derecha
    FL = LegStates() #Para Frontal izquierda
    BR = LegStates() #Pata Trasera derecha
    BL = LegStates() #Pata Trasera izquierda


# ==========================
# Clase principal de IK
# ==========================
class InverseKinematics():
    def __init__(self):
      self.L1 = 86.2 #Hip -> Rodilla(Medir exactamente)
      self.L2 = 150 #Rodilla -> Tobillo(Medir exactamente)
      self.L3 = 150 #Tobillo -> Pie(Medir exactamente)
      self.BODY_LENGTH = 300 #Dimensiones del cuerpo, cambiar
      self.BODY_WIDTH  = 172 #Dimensiones del cuerpo
      self.MIN_ANG_L2L3 = 30 #Define los ángulos mínimos y máximos permitidos entre los segmentos L2 y L3.
      self.MAX_ANG_L2L3 = 170 # Define los ángulos mínimos y máximos permitidos entre los segmentos L2 y L3.
      self.MIN_LEG_R   = np.sqrt(self.L1**2 + (self.L2**2 + self.L3**2 - 2*self.L2*self.L3*np.cos(np.deg2rad(self.MIN_ANG_L2L3)))) #Rango mínimo
      self.MAX_LEG_R   = np.sqrt(self.L1**2 + (self.L2**2 + self.L3**2 - 2*self.L2*self.L3*np.cos(np.deg2rad(self.MAX_ANG_L2L3)))) #Rango máximo

      #Matrices de rotación

      self.M_R = np.array([1, 1, 1]) #Derecha
      self.M_L = np.array([1, -1, 1]) #Izquierda, cambia Y
      self.M_F = np.array([1, 1, 1]) #Adelante
      self.M_B = np.array([-1, 1, 1]) #Atrás, cambia X
      self.BODY_SCALE = np.array([self.BODY_LENGTH/2, self.BODY_WIDTH/2, 0]) #Traslación Frame body -> Piernas

      self.legState = Legs() #Instancia de la clase de las piernas

      self.singularity = [0,0,0,0] #Estado de singularidad

    def rotMat(self, eularAng):
      """
            Input: Ángulos de euler de orientación RPY: np array [roll, pitch, yaw]
            return : Matriz de rotación
            Función: Sirve para adaptar la IK a posiciones inclinadas del cuerpo
      """
      M11 = np.cos(eularAng[1])*np.cos(eularAng[2])
      M12 = np.sin(eularAng[0])*np.sin(eularAng[1])*np.cos(eularAng[2]) - np.cos(eularAng[0])*np.sin(eularAng[2])
      M13 = np.cos(eularAng[0])*np.sin(eularAng[1])*np.cos(eularAng[2]) + np.sin(eularAng[0])*np.sin(eularAng[2])

      M21 = np.cos(eularAng[1])*np.sin(eularAng[2])
      M22 = np.sin(eularAng[0])*np.sin(eularAng[1])*np.sin(eularAng[2]) + np.cos(eularAng[0])*np.cos(eularAng[2])
      M23 = np.cos(eularAng[0])*np.sin(eularAng[1])*np.sin(eularAng[2]) - np.sin(eularAng[0])*np.cos(eularAng[2])

      M31 = -np.sin(eularAng[1])
      M32 = np.sin(eularAng[0])*np.cos(eularAng[1])
      M33 = np.cos(eularAng[0])*np.cos(eularAng[1])
      rotMat = np.array([ [M11, M12, M13], [M21, M22, M23], [M31, M32, M33] ])
      return rotMat

    def get_joint_angles(self, coord):
      """
            coord: arreglo np con la coordenada deseada -> np.array([x, y, z])
            return: arreglo np con los ángulos de servomotores -> np.array([th0, th1, th2])
      """
      joint_ang = np.zeros([3])
      if self.is_singularity(coord):
        pass
      else:
        r_yz = np.sqrt(coord[1]**2 + coord[2]**2)
        alpha = np.arccos(coord[1]/r_yz)
        th0 = np.arccos(self.L1/r_yz) - alpha #Cálculo q1
        if not np.isnan(th0):
          joint_ang[0] = th0

        a = 2*self.L2*r_yz*np.sin(joint_ang[0]+ alpha)
        b = 2*self.L2*coord[0] #x es longitud de zancada
        c = coord[0]**2 + self.L2**2 - self.L3**2 + (a/(2*self.L2))**2
        d = np.sqrt(a**2 + b**2)
        beta = np.arccos(a/d)

        if (coord[0] < 0 ):
          beta = -beta
        if a/d > 1 or a/d < -1 or c/d >1 or c/d < -1:
          pass
        else:
          th1 = beta + np.arcsin(c/d) #Cálculo q2
          if not np.isnan(th1):
            joint_ang[1] = th1

          th2 = np.arccos((coord[0] + self.L2*np.cos(joint_ang[1]))/self.L3) #Cálculo q3
          if not np.isnan(th2):
            joint_ang[2] = th2
      return joint_ang

    def get_FR_joint_angles(self, coord, eularAng):

        """
        Calcula los ángulos articulares de la pata frontal derecha (FR) del robot
        a partir de una coordenada objetivo en el espacio y la orientación actual del cuerpo.

        Este método realiza una serie de transformaciones geométricas para expresar la posición
        del punto objetivo en el marco de referencia de la pata FR. Aplica primero las
        transformaciones de traslación y rotación según el frame del cuerpo y luego verifica si la
        posición resultante cae en una región de singularidad cinemática.

        Si se detecta una singularidad, conserva los ángulos articulares previos para evitar
        comportamientos erráticos. Si no hay singularidad, calcula los nuevos ángulos articulares
        mediante la cinemática inversa.

        Parameters
        ----------
        coord : np.ndarray
            Coordenada objetivo de la pata FR expresada en el frame de la pata [x, y, z].
        eularAng : np.ndarray
            Ángulos de Euler del cuerpo [roll, pitch, yaw] en radianes.

        Returns
        -------
        np.ndarray
            Vector de ángulos articulares [q1, q2, q3] para la pata frontal derecha.
        """

        # Concatenación de transfotmaciones
        translate_FR = self.BODY_SCALE*self.M_F*self.M_R #Define Primer Frame para la pata FR
        # Aplica transformación IZQ/DER (según la pata)
        # -> Traslada la coordenada para expresarla en el frame central del cuerpo
        # -> Aplica la rotación del cuerpo (roll, pitch, yaw)
        # -> Devuelve la coordenada al origen de la pata, pero ya rotada
        # -> Reaplica la transformación IZQ/DER en ese frame
        coord_ = (np.dot((coord * self.M_R + translate_FR), self.rotMat(eularAng)) - translate_FR) * self.M_R
        # Verificación de singularidad
        if self.is_singularity(coord_):
                self.singularity[0] = True
        else:
            self.singularity[0] = False
        #  Verificación de singularidad general
        if any(self.singularity):
            self.legState.FR.now_angles = self.legState.FR.prev_angles
        # Si no hay ninguna singularidad, proceder con el cálculo de ángulos y actualizar y establecer singularidad en False
        else:
            self.legState.FR.now_angles = self.get_joint_angles(coord_)
            self.legState.FR.prev_angles = self.legState.FR.now_angles
            self.singularity[0] = False
        return self.legState.FR.now_angles

    def get_FL_joint_angles(self, coord, eularAng):

        """
        Ver documentación de get_FR_joint_angles
        Esta función realiza el mismo procedimiento, pero para la pata frontal izquierda (FL).
        """
        # Concatenación de transfotmaciones
        translate_FL = self.BODY_SCALE*self.M_F*self.M_L #Define Primer Frame para la pata FL
        # Aplica transformación IZQ/DER (según la pata)
        # -> Traslada la coordenada para expresarla en el frame central del cuerpo
        # -> Aplica la rotación del cuerpo (roll, pitch, yaw)
        # -> Devuelve la coordenada al origen de la pata, pero ya rotada
        # -> Reaplica la transformación IZQ/DER en ese frame
        coord_ = (np.dot((coord * self.M_L + translate_FL), self.rotMat(eularAng)) - translate_FL) * self.M_L
        # Verificación de singularidad
        if self.is_singularity(coord_):
                self.singularity[1] = True
        else:
            self.singularity[1] = False
        #  Verificación de singularidad general
        if any(self.singularity):
            self.legState.FL.now_angles = self.legState.FL.prev_angles
        # Si no hay ninguna singularidad, proceder con el cálculo de ángulos y actualizar y establecer singularidad en False
        else:
            self.legState.FL.now_angles = self.get_joint_angles(coord_)
            self.legState.FL.prev_angles = self.legState.FL.now_angles
            self.singularity[1] = False
        return self.legState.FL.now_angles


    def get_BR_joint_angles(self, coord, eularAng):

        """
        Ver documentación de get_FR_joint_angles
        Esta función realiza el mismo procedimiento, pero para la pata trasera derecha (BR).
        """
        # Concatenación de transfotmaciones
        translate_BR = self.BODY_SCALE*self.M_B*self.M_R #Define Primer Frame para la pata BR
        coord_ = (np.dot((coord * self.M_R + translate_BR), self.rotMat(eularAng)) - translate_BR) * self.M_R
        # Aplica transformación IZQ/DER (según la pata)
        # -> Traslada la coordenada para expresarla en el frame central del cuerpo
        # -> Aplica la rotación del cuerpo (roll, pitch, yaw)
        # -> Devuelve la coordenada al origen de la pata, pero ya rotada
        # -> Reaplica la transformación IZQ/DER en ese frame
        # Verificación de singularidad
        if self.is_singularity(coord_):
                self.singularity[2] = True
        else:
            self.singularity[2] = False
         #  Verificación de singularidad general
        if any(self.singularity):
            self.legState.BR.now_angles = self.legState.BR.prev_angles
        # Si no hay ninguna singularidad, proceder con el cálculo de ángulos y actualizar y establecer singularidad en False
        else:
            self.legState.BR.now_angles = self.get_joint_angles(coord_)
            self.legState.BR.prev_angles = self.legState.BR.now_angles
            self.singularity[2] = False
        return self.legState.BR.now_angles

    def get_BL_joint_angles(self, coord, eularAng):

        """
        Ver documentación de get_FR_joint_angles
        Esta función realiza el mismo procedimiento, pero para la pata trasera izquierda (BL).
        """
        # Concatenación de transfotmaciones
        translate_FL = self.BODY_SCALE*self.M_B*self.M_L #Define Primer Frame para la pata BL
        coord_ = (np.dot((coord * self.M_L + translate_FL), self.rotMat(eularAng)) - translate_FL) * self.M_L
        # Aplica transformación IZQ/DER (según la pata)
        # -> Traslada la coordenada para expresarla en el frame central del cuerpo
        # -> Aplica la rotación del cuerpo (roll, pitch, yaw)
        # -> Devuelve la coordenada al origen de la pata, pero ya rotada
        # -> Reaplica la transformación IZQ/DER en ese frame
        # Verificación de singularidad
        if self.is_singularity(coord_):
                self.singularity[3] = True
        else:
            self.singularity[3] = False
        #  Verificación de singularidad general
        if any(self.singularity):
            self.legState.BL.now_angles = self.legState.BL.prev_angles
        # Si no hay ninguna singularidad, proceder con el cálculo de ángulos y actualizar y establecer singularidad en False
        else:
            self.legState.BL.now_angles = self.get_joint_angles(coord_)
            self.legState.BL.prev_angles = self.legState.BL.now_angles
            self.singularity[3] = False
        return self.legState.BL.now_angles

    def is_singularity(self, coord):
        r = np.sqrt(coord[0]**2 + coord[1]**2 + coord[2]**2)
        if r >= self.MIN_LEG_R and r <= self.MAX_LEG_R:
            return False
        else:
            return True

    #     self.legState = Legs()
    #     self.singularity = [False, False, False, False]

    # def get_joint_angles(self, coord):
    #     """
    #     coord: np.array([x, y, z])
    #     Retorna: np.array([q1, q2, q3])
    #     """
    #     try:
    #         q = mgi_quadruped(coord[0], coord[1], coord[2])
    #     except:
    #         q = np.array([0.0, 0.0, 0.0])
    #     return q

    # def get_FR_joint_angles(self, coord):
    #     self.legState.FR.now_angles = self.get_joint_angles(coord)
    #     self.legState.FR.prev_angles = self.legState.FR.now_angles
    #     return self.legState.FR.now_angles

    # def get_FL_joint_angles(self, coord):
    #     self.legState.FL.now_angles = self.get_joint_angles(coord)
    #     self.legState.FL.prev_angles = self.legState.FL.now_angles
    #     return self.legState.FL.now_angles

    # def get_BR_joint_angles(self, coord):
    #     self.legState.BR.now_angles = self.get_joint_angles(coord)
    #     self.legState.BR.prev_angles = self.legState.BR.now_angles
    #     return self.legState.BR.now_angles

    # def get_BL_joint_angles(self, coord):
    #     self.legState.BL.now_angles = self.get_joint_angles(coord)
    #     self.legState.BL.prev_angles = self.legState.BL.now_angles
    #     return self.legState.BL.now_angles